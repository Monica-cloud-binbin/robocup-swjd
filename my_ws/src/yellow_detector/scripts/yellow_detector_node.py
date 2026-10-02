#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""使用 OpenCV HSV 颜色分割检测黄色气球的 ROS1 节点。"""

from collections import deque
import math

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
        # 颜色阈值保持不变，使用形状和最近帧命中数抑制误检。
        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.0015))
        self.max_area_ratio = float(rospy.get_param("~max_area_ratio", 0.70))
        self.min_aspect_ratio = float(rospy.get_param("~min_aspect_ratio", 0.65))
        self.max_aspect_ratio = float(rospy.get_param("~max_aspect_ratio", 1.40))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.60))
        self.min_extent = float(rospy.get_param("~min_extent", 0.55))
        self.min_solidity = float(rospy.get_param("~min_solidity", 0.88))
        self.rectangle_reject_extent = float(
            rospy.get_param("~rectangle_reject_extent", 0.84)
        )
        self.confirm_window = max(1, int(rospy.get_param("~confirm_window", 5)))
        requested_hits = int(rospy.get_param("~confirm_hits", 4))
        self.confirm_hits = min(max(1, requested_hits), self.confirm_window)
        if requested_hits != self.confirm_hits:
            rospy.logwarn(
                "confirm_hits=%d 已限制到 [1, confirm_window]，使用 %d",
                requested_hits,
                self.confirm_hits,
            )
        self.detect_history = deque(maxlen=self.confirm_window)

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

        # 飞控现有接口：只在多帧确认成功时发布 PointStamped。
        self.mission_pub = rospy.Publisher(
            self.mission_topic, PointStamped, queue_size=10
        )
        # 只有多帧确认成功时发布完整检测结果。
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

    def evaluate_contour(self, contour, frame_width, frame_height):
        """统一几何筛选；返回指标和首个拒绝原因，异常轮廓不影响其他目标。"""
        result = {"accepted": False, "reason": "AREA", "area": 0.0}
        try:
            area = float(cv2.contourArea(contour))
            if not math.isfinite(area):
                return result
            result["area"] = area
            if area <= 0.0 or area < self.min_area:
                return result

            result["reason"] = "AREA_RATIO"
            frame_area = float(frame_width) * float(frame_height)
            if frame_area <= 0.0 or not math.isfinite(frame_area):
                return result
            area_ratio = area / frame_area
            result["area_ratio"] = area_ratio
            if not self.min_area_ratio <= area_ratio <= self.max_area_ratio:
                return result

            result["reason"] = "ASPECT"
            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                return result
            aspect_ratio = float(w) / float(h)
            result["bbox"] = (x, y, w, h)
            result["aspect_ratio"] = aspect_ratio
            if not self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio:
                return result

            result["reason"] = "CIRCULARITY"
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0 or not math.isfinite(perimeter):
                return result
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            result["circularity"] = circularity
            if circularity < self.min_circularity:
                return result

            result["reason"] = "EXTENT"
            extent = area / float(w * h)
            result["extent"] = extent
            if extent < self.min_extent:
                return result

            result["reason"] = "SOLIDITY"
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            if hull_area <= 0.0 or not math.isfinite(hull_area):
                return result
            solidity = area / hull_area
            result["solidity"] = solidity
            if solidity < self.min_solidity:
                return result

            result["reason"] = "RECTANGLE"
            approx = cv2.approxPolyDP(contour, 0.03 * perimeter, True)
            # 只排除四边形且填充程度高的区域，不能仅凭四个顶点拒绝。
            if len(approx) == 4 and extent > self.rectangle_reject_extent:
                return result

            result["accepted"] = True
            result["reason"] = None
        except (cv2.error, TypeError, ValueError, OverflowError) as exc:
            rospy.logwarn_throttle(2.0, "跳过异常黄色轮廓: %s", str(exc))
        return result

    def _mark_no_detection(self):
        """无效输入也计作一次未命中，避免旧历史跨过坏帧直接确认。"""
        self.detect_history.append(False)

    def image_callback(self, image_msg):
        """处理一帧图像；只有当前候选且最近窗口命中达标才发布任务位置。"""
        try:
            # cv_bridge 根据原始 encoding 转换成 RGB；阈值按 RGB -> HSV 处理。
            rgb_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
        except (CvBridgeError, cv2.error, TypeError, ValueError) as exc:
            rospy.logerr_throttle(2.0, "图像转换失败: %s", str(exc))
            self._mark_no_detection()
            return

        if (
            not isinstance(rgb_image, np.ndarray)
            or rgb_image.size == 0
            or rgb_image.ndim != 3
            or rgb_image.shape[2] != 3
        ):
            rospy.logerr_throttle(2.0, "收到空图像或无效 RGB 图像，跳过本帧")
            self._mark_no_detection()
            return

        height, width = rgb_image.shape[:2]
        try:
            # 保留原有颜色分割、开运算和闭运算。
            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            contour_result = cv2.findContours(
                mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            contours = contour_result[-2]
            debug_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        except (cv2.error, TypeError, ValueError) as exc:
            rospy.logerr_throttle(2.0, "图像处理失败: %s", str(exc))
            self._mark_no_detection()
            return

        # 单次遍历选最大合格候选。
        best = None
        for contour in contours:
            evaluated = self.evaluate_contour(contour, width, height)
            if evaluated["accepted"]:
                if best is None or evaluated["area"] > best["area"]:
                    best = evaluated

        current_detected = best is not None
        self.detect_history.append(current_detected)
        stable_hits = sum(self.detect_history)
        stable_detected = current_detected and stable_hits >= self.confirm_hits

        if best is not None:
            x, y, box_width, box_height = best["bbox"]
            xmin, ymin = int(x), int(y)
            xmax, ymax = int(x + box_width), int(y + box_height)
            x_pixel = int(x + box_width // 2)
            y_pixel = int(y + box_height // 2)
            area = best["area"]
            area_ratio = min(max(best["area_ratio"], 0.0), 1.0)

            # BGR：候选橙色，多帧确认绿色。
            color = (0, 255, 0) if stable_detected else (0, 165, 255)
            cv2.rectangle(
                debug_image,
                (xmin, ymin),
                (min(xmax - 1, width - 1), min(ymax - 1, height - 1)),
                color,
                2,
            )
            cv2.circle(debug_image, (x_pixel, y_pixel), 4, (0, 0, 255), -1)
            label = (
                "A={:.0f} AR={:.2f} C={:.2f} E={:.2f} S={:.2f} H={}/{}"
            ).format(
                area,
                best["aspect_ratio"],
                best["circularity"],
                best["extent"],
                best["solidity"],
                stable_hits,
                self.confirm_hits,
            )
            # 适配小分辨率，避免指标文本从画面右侧溢出。
            font = cv2.FONT_HERSHEY_SIMPLEX
            text_width = cv2.getTextSize(label, font, 0.55, 1)[0][0]
            font_scale = min(0.55, 0.55 * max(1, width - 8) / max(1, text_width))
            label_width = cv2.getTextSize(label, font, font_scale, 1)[0][0]
            text_x = max(0, min(xmin, width - label_width - 4))
            text_y = min(height - 1, max(18, ymin - 8))
            cv2.putText(
                debug_image, label, (text_x, text_y), font,
                font_scale, color, 1, cv2.LINE_AA,
            )

            if stable_detected:
                detection = YellowBalloonDetection()
                detection.header = image_msg.header
                detection.detected = True
                detection.x_pixel = x_pixel
                detection.y_pixel = y_pixel
                detection.xmin = xmin
                detection.ymin = ymin
                detection.xmax = xmax
                detection.ymax = ymax
                detection.area = area
                detection.area_ratio = area_ratio
                self.result_pub.publish(detection)

                # 保留主控字段：x/y=框中心像素，z=轮廓面积占整帧比例。
                # 无目标或尚未确认时不发布，继续使用主控原有超时机制。
                mission_msg = PointStamped()
                mission_msg.header = image_msg.header
                mission_msg.point.x = float(x_pixel)
                mission_msg.point.y = float(y_pixel)
                mission_msg.point.z = float(area_ratio)
                self.mission_pub.publish(mission_msg)

            log_details = (
                "center=(%d,%d), bbox=(%d,%d,%d,%d), area=%.0f px, "
                "ratio=%.4f, AR=%.2f, C=%.2f, E=%.2f, S=%.2f, hits=%d/%d"
            ) % (
                x_pixel, y_pixel, xmin, ymin, xmax, ymax,
                area, area_ratio, best["aspect_ratio"], best["circularity"],
                best["extent"], best["solidity"], stable_hits, self.confirm_hits,
            )
            if stable_detected:
                rospy.loginfo_throttle(2.0, "绿框确认黄色气球: %s", log_details)
            else:
                rospy.loginfo_throttle(2.0, "橙框候选黄色气球: %s", log_details)
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

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = image_msg.header
            self.debug_pub.publish(debug_msg)
        except (CvBridgeError, cv2.error, TypeError, ValueError) as exc:
            rospy.logerr_throttle(2.0, "调试图像转换失败: %s", str(exc))


def main():
    rospy.init_node("yellow_detector", anonymous=False)
    YellowDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
