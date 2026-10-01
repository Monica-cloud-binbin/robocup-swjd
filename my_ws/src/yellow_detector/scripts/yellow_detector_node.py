#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""使用 OpenCV HSV 颜色分割检测黄色气球的 ROS1 节点。"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image

from yellow_detector.msg import YellowBalloonDetection


class YellowDetectorNode:
    """订阅相机图像，发布检测结果和带框调试图像。"""

    def __init__(self):
        self.bridge = CvBridge()

        # ROS 参数：输入与输出 topic。
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.mission_topic = rospy.get_param(
            "~mission_topic", "/yellow_balloon_position"
        )
        self.result_topic = rospy.get_param(
            "~result_topic", "/yellow_detector/detection"
        )
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/yellow_detector/debug_image"
        )

        # OpenCV 的 HSV 色相 H 范围为 0..179，S 和 V 范围为 0..255。
        self.h_min = self._bounded_int_param("~h_min", 18, 0, 179)
        self.h_max = self._bounded_int_param("~h_max", 38, 0, 179)
        self.s_min = self._bounded_int_param("~s_min", 80, 0, 255)
        self.s_max = self._bounded_int_param("~s_max", 255, 0, 255)
        self.v_min = self._bounded_int_param("~v_min", 80, 0, 255)
        self.v_max = self._bounded_int_param("~v_max", 255, 0, 255)

        if self.h_min > self.h_max:
            self.h_min, self.h_max = self.h_max, self.h_min
            rospy.logwarn("h_min 大于 h_max，已交换两个参数的值")
        if self.s_min > self.s_max:
            self.s_min, self.s_max = self.s_max, self.s_min
            rospy.logwarn("s_min 大于 s_max，已交换两个参数的值")
        if self.v_min > self.v_max:
            self.v_min, self.v_max = self.v_max, self.v_min
            rospy.logwarn("v_min 大于 v_max，已交换两个参数的值")

        self.min_area = max(0.0, float(rospy.get_param("~min_area", 500.0)))
        kernel_size = max(1, int(rospy.get_param("~morph_kernel_size", 5)))
        if kernel_size % 2 == 0:
            kernel_size += 1
            rospy.logwarn("形态学核尺寸必须为奇数，已自动加一")
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )

        self.lower_hsv = np.array(
            [self.h_min, self.s_min, self.v_min], dtype=np.uint8
        )
        self.upper_hsv = np.array(
            [self.h_max, self.s_max, self.v_max], dtype=np.uint8
        )

        # 飞控现有接口：只在检测成功时发布 PointStamped。
        self.mission_pub = rospy.Publisher(
            self.mission_topic, PointStamped, queue_size=10
        )
        # 完整检测状态每帧发布，未检测到时 detected=false，其他数值为 0。
        self.result_pub = rospy.Publisher(
            self.result_topic, YellowBalloonDetection, queue_size=10
        )
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)

        # 大图像使用较大的接收缓冲区；队列为 1，优先处理最新帧。
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        rospy.loginfo("黄色气球检测节点已启动")
        rospy.loginfo("输入图像: %s", self.image_topic)
        rospy.loginfo("飞控结果: %s (geometry_msgs/PointStamped)", self.mission_topic)
        rospy.loginfo("完整结果: %s (yellow_detector/YellowBalloonDetection)", self.result_topic)
        rospy.loginfo("调试图像: %s", self.debug_topic)
        rospy.loginfo(
            "HSV 阈值 H[%d,%d] S[%d,%d] V[%d,%d]，最小面积 %.1f px",
            self.h_min,
            self.h_max,
            self.s_min,
            self.s_max,
            self.v_min,
            self.v_max,
            self.min_area,
        )

    @staticmethod
    def _bounded_int_param(name, default, lower, upper):
        """读取整数参数并限制在 OpenCV HSV 的合法范围内。"""
        value = int(rospy.get_param(name, default))
        bounded = min(max(value, lower), upper)
        if value != bounded:
            rospy.logwarn("参数 %s=%d 超出范围，已限制为 %d", name, value, bounded)
        return bounded

    def image_callback(self, image_msg):
        """处理一帧图像并发布任务接口、完整结果和调试图像。"""
        try:
            # cv_bridge 根据原始 encoding 转换成 RGB；阈值分割按 RGB -> HSV 处理。
            rgb_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, "图像转换失败: %s", str(exc))
            return

        height, width = rgb_image.shape[:2]
        if height <= 0 or width <= 0:
            rospy.logerr_throttle(2.0, "收到空图像，跳过本帧")
            return

        # 彩色空间转换、HSV 阈值二值化和形态学去噪。
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # 查找外轮廓，并从达到面积门限的轮廓中选出最大区域。
        contour_result = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contour_result[-2]
        candidates = [
            (float(cv2.contourArea(contour)), contour)
            for contour in contours
            if cv2.contourArea(contour) >= self.min_area
        ]
        best = max(candidates, key=lambda item: item[0]) if candidates else None

        detection = YellowBalloonDetection()
        detection.header = image_msg.header
        # ROS debug_image 发布 bgr8，因此为绘图创建一份 BGR 图像。
        debug_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        if best is not None:
            area, contour = best
            x, y, box_width, box_height = cv2.boundingRect(contour)
            xmin = int(x)
            ymin = int(y)
            xmax = int(x + box_width)
            ymax = int(y + box_height)
            x_pixel = int(x + box_width // 2)
            y_pixel = int(y + box_height // 2)
            area_ratio = min(max(area / float(width * height), 0.0), 1.0)

            detection.detected = True
            detection.x_pixel = x_pixel
            detection.y_pixel = y_pixel
            detection.xmin = xmin
            detection.ymin = ymin
            detection.xmax = xmax
            detection.ymax = ymax
            detection.area = area
            detection.area_ratio = area_ratio

            # 绘制绿色框和目标名称；HSV 检测没有分类置信度，因此标注像素面积。
            cv2.rectangle(
                debug_image,
                (xmin, ymin),
                (min(xmax - 1, width - 1), min(ymax - 1, height - 1)),
                (0, 255, 0),
                2,
            )
            cv2.circle(debug_image, (x_pixel, y_pixel), 4, (0, 0, 255), -1)
            label = "yellow_balloon area={:.0f}px".format(area)
            text_y = max(22, ymin - 8)
            cv2.putText(
                debug_image,
                label,
                (xmin, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            # 兼容现有飞控：x/y 是像素中心，z 是画面面积占比 [0, 1]。
            # 不在未检测帧发布该消息，让任务节点按自己的超时逻辑判断目标丢失。
            mission_msg = PointStamped()
            mission_msg.header = image_msg.header
            mission_msg.point.x = float(x_pixel)
            mission_msg.point.y = float(y_pixel)
            mission_msg.point.z = float(area_ratio)
            self.mission_pub.publish(mission_msg)

            rospy.loginfo_throttle(
                2.0,
                "发现黄色区域: center=(%d,%d), bbox=(%d,%d,%d,%d), "
                "area=%.0f px, ratio=%.4f",
                x_pixel,
                y_pixel,
                xmin,
                ymin,
                xmax,
                ymax,
                area,
                area_ratio,
            )
        else:
            cv2.putText(
                debug_image,
                "yellow_balloon: not found",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # 无论是否检测到目标，每帧都发布带状态的强类型检测结果。
        self.result_pub.publish(detection)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = image_msg.header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, "调试图像转换失败: %s", str(exc))


def main():
    rospy.init_node("yellow_detector", anonymous=False)
    YellowDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
