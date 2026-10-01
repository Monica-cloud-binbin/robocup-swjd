#include "object_position.h"
int asd;
MoveObject::MoveObject() : it_(nh_)
{
    // 发布目标在图像中的坐标
    position_pub = nh_.advertise<geometry_msgs::PointStamped>("/object_position", 10);
    // 订阅YOLO检测结果
    Object_sub = nh_.subscribe("/yolov8/BoundingBoxes", 1, &MoveObject::ObjectCallback, this);
}

MoveObject::~MoveObject()
{
    ROS_INFO("MoveObject node stopped.");
}

void MoveObject::ObjectCallback(const yolov8_ros_msgs::BoundingBoxes &object_msg)
{
float bbox_ratio = 0.0f;      // 宽高比
float bbox_width = 0.0f;
float bbox_height = 0.0f;
    if (object_msg.bounding_boxes.empty())
    {
        ROS_INFO("No object detected.");
        return;
    }

    // 取第一个检测到的目标框（你可以改为根据类别筛选）
    const auto &box = object_msg.bounding_boxes[0];
    std::string obj_class = box.Class;
    float confidence = box.probability; //获取识别相似度
    
    if(box.probability<0.8)
    {
    	ROS_INFO("Low Probability");
    	return;
    }
    float width = box.xmax - box.xmin;
    float height = box.ymax - box.ymin;
    ROS_INFO("width=%f,height=%f",width,height);
    if (height > 0)
    {
    	bbox_ratio = height / width;
    }
    // 计算目标中心像素坐标
    int center_x = (box.xmin + box.xmax) / 2;
    int center_y = (box.ymin + box.ymax) / 2;
    int center_z = bbox_ratio*100;
    // 发布坐标
    geometry_msgs::PointStamped point_msg;
    point_msg.header.stamp = ros::Time::now();
    point_msg.header.frame_id = "rgb_frame";  // 可以换成相机的frame
    point_msg.point.x = center_x;
    point_msg.point.y = center_y;
    point_msg.point.z = center_z;  // 不使用深度，固定为0

    position_pub.publish(point_msg);

    ROS_INFO("Detected: %s | x=%d, y=%d", obj_class.c_str(), center_x, center_y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_object_position");
    MoveObject move_object;
    ros::spin();
    return 0;
}
