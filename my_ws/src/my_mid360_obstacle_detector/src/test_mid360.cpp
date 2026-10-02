#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include <vector>
#include <algorithm>

// 全局变量
ros::Publisher obstacle_pub;
double front_safe_distance = 1.0;
double side_safe_distance = 0.8;
double rear_safe_distance = 1.2;

// 激光雷达回调函数
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // 获取激光雷达参数
    int num_points = msg->ranges.size();
    float angle_min = msg->angle_min;      // 最小角度（弧度）
    float angle_max = msg->angle_max;      // 最大角度（弧度）
    float angle_increment = msg->angle_increment;  // 角度增量
    
    // 初始化各个方向的最小距离
    float front_min = 999.0;
    float left_min = 999.0;
    float right_min = 999.0;
    float rear_min = 999.0;
    
    // 遍历所有激光点
    for (int i = 0; i < num_points; i++) {
        float range = msg->ranges[i];
        float angle = angle_min + i * angle_increment;
        
        // 忽略无效值
        if (std::isinf(range) || std::isnan(range) || range < msg->range_min) {
            continue;
        }
        
        // 将角度归一化到 [-π, π]
        if (angle > M_PI) angle -= 2 * M_PI;
        if (angle < -M_PI) angle += 2 * M_PI;
        
        // 根据角度分类到不同区域
        if (std::abs(angle) <= M_PI/4) {  // 前方 ±45°
            if (range < front_min) front_min = range;
        }
        else if (angle > M_PI/4 && angle <= 3*M_PI/4) {  // 右侧 45°-135°
            if (range < right_min) right_min = range;
        }
        else if (angle < -M_PI/4 && angle >= -3*M_PI/4) {  // 左侧 -45°到-135°
            if (range < left_min) left_min = range;
        }
        else {  // 后方 135°到-135°
            if (range < rear_min) rear_min = range;
        }
    }
    
    // 创建障碍物距离消息
    std_msgs::Float32MultiArray obstacle_msg;
    obstacle_msg.data.resize(4);
    obstacle_msg.data[0] = front_min;   // 前方最小距离
    obstacle_msg.data[1] = right_min;   // 右侧最小距离
    obstacle_msg.data[2] = rear_min;    // 后方最小距离
    obstacle_msg.data[3] = left_min;    // 左侧最小距离
    
    // 发布障碍物距离信息
    obstacle_pub.publish(obstacle_msg);
    
    // 打印障碍物距离信息（可选，用于调试）
    ROS_INFO_THROTTLE(0.5, "Obstacle distances - Front: %.2fm, Right: %.2fm, Rear: %.2fm, Left: %.2fm", 
                     front_min, right_min, rear_min, left_min);
    
    // 安全检查：如果任何方向距离小于安全距离，发出警告
        ROS_WARN_THROTTLE(0.5, "Front obstacle too close: %.2fm (safe: %.2fm)", front_min, front_safe_distance);
        ROS_WARN_THROTTLE(0.5, "Right obstacle too close: %.2fm (safe: %.2fm)", right_min, side_safe_distance);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mid360_obstacle_detector");
    ros::NodeHandle nh;
    
    // 从参数服务器读取安全距离参数
    nh.param("front_safe_distance", front_safe_distance, 1.0);
    nh.param("side_safe_distance", side_safe_distance, 0.8);
    nh.param("rear_safe_distance", rear_safe_distance, 1.2);
    
    // 订阅Mid-360激光雷达数据
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserCallback);
    
    // 发布障碍物距离信息
    obstacle_pub = nh.advertise<std_msgs::Float32MultiArray>("/obstacle_distances", 10);
    
    ROS_INFO("Mid-360 Obstacle Detector started");
    ROS_INFO("Safe distances - Front: %.2fm, Side: %.2fm, Rear: %.2fm", 
             front_safe_distance, side_safe_distance, rear_safe_distance);
    ROS_INFO("Publishing obstacle distances to /obstacle_distances");
    
    ros::spin();
    
    return 0;
}
