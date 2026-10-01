#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import tf

class RadarObstacleVisualizer:
    def __init__(self):
        rospy.init_node('radar_obstacle_visualizer', anonymous=True)
        
        # 参数设置
        self.obstacle_threshold = 5  # 障碍物检测阈值（米）
        self.cluster_threshold = 0.3   # 障碍物聚类阈值（米）
        self.min_points_for_cluster = 3  # 最小聚类点数
        
        # 订阅激光雷达数据
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        
        # 发布器 - 用于可视化
        self.obstacle_points_pub = rospy.Publisher("/obstacle_points", PointStamped, queue_size=10)
        self.obstacle_markers_pub = rospy.Publisher("/obstacle_markers", MarkerArray, queue_size=10)
        self.obstacle_poses_pub = rospy.Publisher("/obstacle_poses", PoseArray, queue_size=10)
        self.closest_obstacle_pub = rospy.Publisher("/closest_obstacle", PointStamped, queue_size=10)
        
        # 障碍物数据存储
        self.obstacles = []
        self.clusters = []
        self.closest_obstacle = None
        
        rospy.loginfo("雷达障碍物可视化节点已启动")
        rospy.loginfo("障碍物检测阈值: %.1f米", self.obstacle_threshold)
        rospy.loginfo("聚类阈值: %.1f米", self.cluster_threshold)

    def laser_callback(self, msg):
        """激光雷达数据回调函数"""
        obstacles = self.detect_obstacles(msg)
        clusters = self.cluster_obstacles(obstacles)
        self.visualize_obstacles(obstacles, clusters, msg.header)
        
        # 打印障碍物信息
        self.print_obstacle_info(clusters)

    def detect_obstacles(self, scan_msg):
        """检测障碍物点"""
        obstacles = []
        
        if len(scan_msg.ranges) == 0:
            return obstacles
        
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        for i, distance in enumerate(scan_msg.ranges):
            # 跳过无效数据
            if (distance < scan_msg.range_min or 
                distance > scan_msg.range_max or 
                math.isinf(distance) or 
                math.isnan(distance)):
                continue
            
            # 检测障碍物（距离小于阈值）
            if distance < self.obstacle_threshold:
                # 计算障碍物在机器人坐标系中的位置
                angle = angle_min + i * angle_increment
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                
                obstacles.append({
                    'x': x,
                    'y': y,
                    'distance': distance,
                    'angle': math.degrees(angle),
                    'index': i
                })
        
        return obstacles

    def cluster_obstacles(self, obstacles):
        """对障碍物进行聚类"""
        if not obstacles:
            return []
        
        clusters = []
        used_points = set()
        
        for i, obs in enumerate(obstacles):
            if i in used_points:
                continue
                
            # 创建新聚类
            cluster = [obs]
            used_points.add(i)
            
            # 寻找邻近点
            for j, other_obs in enumerate(obstacles):
                if j in used_points:
                    continue
                
                # 计算两点间距离
                dist = math.sqrt((obs['x'] - other_obs['x'])**2 + 
                               (obs['y'] - other_obs['y'])**2)
                
                if dist < self.cluster_threshold:
                    cluster.append(other_obs)
                    used_points.add(j)
            
            # 只保留有足够点的聚类
            if len(cluster) >= self.min_points_for_cluster:
                clusters.append(cluster)
        
        return clusters

    def visualize_obstacles(self, obstacles, clusters, header):
        """可视化障碍物"""
        self.visualize_raw_points(obstacles, header)
        self.visualize_clusters(clusters, header)
        self.visualize_closest_obstacle(obstacles, header)

    def visualize_raw_points(self, obstacles, header):
        """可视化原始障碍物点"""
        pose_array = PoseArray()
        pose_array.header = header
        
        for obstacle in obstacles:
            pose = Pose()
            pose.position.x = obstacle['x']
            pose.position.y = obstacle['y']
            pose.position.z = 0.0
            pose_array.poses.append(pose)
        
        self.obstacle_poses_pub.publish(pose_array)

    def visualize_clusters(self, clusters, header):
        """可视化聚类后的障碍物"""
        marker_array = MarkerArray()
        
        # 清除之前的标记
        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, cluster in enumerate(clusters):
            # 计算聚类中心
            center_x = sum(obs['x'] for obs in cluster) / len(cluster)
            center_y = sum(obs['y'] for obs in cluster) / len(cluster)
            avg_distance = sum(obs['distance'] for obs in cluster) / len(cluster)
            
            # 创建球形标记
            marker = Marker()
            marker.header = header
            marker.ns = "obstacle_clusters"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.0
            
            # 根据距离设置大小和颜色
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # 颜色：越近越红，越远越绿
            intensity = min(1.0, avg_distance / self.obstacle_threshold)
            marker.color.r = 1.0 - intensity
            marker.color.g = intensity
            marker.color.b = 0.0
            marker.color.a = 0.8  # 透明度
            
            marker.lifetime = rospy.Duration(0.5)  # 0.5秒后自动消失
            marker_array.markers.append(marker)
            
            # 添加文字标记显示距离
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "obstacle_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = center_x
            text_marker.pose.position.y = center_y
            text_marker.pose.position.z = 0.5
            
            text_marker.scale.z = 0.2  # 文字大小
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = "%.2fm" % avg_distance
            text_marker.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(text_marker)
        
        self.obstacle_markers_pub.publish(marker_array)

    def visualize_closest_obstacle(self, obstacles, header):
        """可视化最近的障碍物"""
        if not obstacles:
            return
        
        # 找到最近的障碍物
        closest = min(obstacles, key=lambda x: x['distance'])
        
        # 发布最近障碍物点
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.point.x = closest['x']
        point_msg.point.y = closest['y']
        point_msg.point.z = 0.0
        
        self.closest_obstacle_pub.publish(point_msg)
        self.closest_obstacle = closest

    def print_obstacle_info(self, clusters):
        """打印障碍物信息"""
        if not clusters:
            rospy.loginfo_throttle(2, "前方无障碍物")
            return
        
        # 打印聚类信息
        rospy.loginfo("=== 障碍物检测报告 ===")
        rospy.loginfo("检测到 %d 个障碍物区域", len(clusters))
        
        for i, cluster in enumerate(clusters):
            center_x = sum(obs['x'] for obs in cluster) / len(cluster)
            center_y = sum(obs['y'] for obs in cluster) / len(cluster)
            avg_distance = sum(obs['distance'] for obs in cluster) / len(cluster)
            min_distance = min(obs['distance'] for obs in cluster)
            
            # 计算角度（相对于机器人前方）
            angle_deg = math.degrees(math.atan2(center_y, center_x))
            
            rospy.loginfo("障碍物 %d: 距离=%.2fm, 角度=%.1f°, 位置=(%.2f, %.2f), 点数=%d", 
                         i+1, avg_distance, angle_deg, center_x, center_y, len(cluster))
        
        # 打印最近障碍物信息
        if self.closest_obstacle:
            rospy.loginfo("最近障碍物: 距离=%.2fm, 角度=%.1f°, 位置=(%.2f, %.2f)", 
                         self.closest_obstacle['distance'],
                         self.closest_obstacle['angle'],
                         self.closest_obstacle['x'],
                         self.closest_obstacle['y'])
        
        rospy.loginfo("=" * 30)

    def run(self):
        """运行节点"""
        rate = rospy.Rate(1)  # 5Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    try:
        visualizer = RadarObstacleVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
