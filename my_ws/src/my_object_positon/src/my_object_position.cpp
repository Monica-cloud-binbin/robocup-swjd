#include "object_position.h"
int asd;
MoveObject::MoveObject() : it_(nh_)
{
    nh_.param<double>("~confidence_threshold", confidence_threshold_, 0.8);
    nh_.param<int>("~image_width", image_width_, 640);
    nh_.param<int>("~image_height", image_height_, 480);

    // 保留旧接口，供仍使用 PointStamped 的旧主控兼容。
    position_pub = nh_.advertise<geometry_msgs::PointStamped>("/object_position", 10);
    // 新接口保留类别、置信度和完整框信息，供当前主控区分多类目标。
    detection_pub = nh_.advertise<my_object_position::ObjectDetection>(
        "/object_detection", 10);
    // 订阅YOLO检测结果
    Object_sub = nh_.subscribe("/yolov8/BoundingBoxes", 1, &MoveObject::ObjectCallback, this);
}

MoveObject::~MoveObject()
{
    ROS_INFO("MoveObject node stopped.");
}

void MoveObject::ObjectCallback(const yolov8_ros_msgs::BoundingBoxes &object_msg)
{
    if (object_msg.bounding_boxes.empty())
    {
        ROS_INFO_THROTTLE(1.0, "No object detected.");
        return;
    }

    bool legacy_published = false;
    for (const auto &box : object_msg.bounding_boxes)
    {
        if (box.probability < confidence_threshold_)
            continue;

        const float width = static_cast<float>(box.xmax - box.xmin);
        const float height = static_cast<float>(box.ymax - box.ymin);
        if (width <= 0.0f || height <= 0.0f)
        {
            ROS_WARN_THROTTLE(1.0, "Ignoring invalid YOLO bounding box.");
            continue;
        }

        const int center_x = static_cast<int>((box.xmin + box.xmax) / 2);
        const int center_y = static_cast<int>((box.ymin + box.ymax) / 2);
        const float aspect_ratio = height / width;
        const float image_area = static_cast<float>(image_width_ * image_height_);

        my_object_position::ObjectDetection detection_msg;
        detection_msg.header = object_msg.image_header;
        if (detection_msg.header.stamp.isZero())
            detection_msg.header.stamp = ros::Time::now();
        detection_msg.class_name = box.Class;
        detection_msg.confidence = box.probability;
        detection_msg.xmin = box.xmin;
        detection_msg.ymin = box.ymin;
        detection_msg.xmax = box.xmax;
        detection_msg.ymax = box.ymax;
        detection_msg.center_x = center_x;
        detection_msg.center_y = center_y;
        detection_msg.width = width;
        detection_msg.height = height;
        detection_msg.aspect_ratio = aspect_ratio;
        detection_msg.area_ratio = image_area > 0.0f
            ? (width * height) / image_area
            : 0.0f;
        detection_pub.publish(detection_msg);

        // 旧接口只发布第一个通过过滤的框，保持单目标消费者可继续运行。
        if (!legacy_published)
        {
            geometry_msgs::PointStamped point_msg;
            point_msg.header = detection_msg.header;
            point_msg.header.frame_id = "rgb_frame";
            point_msg.point.x = center_x;
            point_msg.point.y = center_y;
            point_msg.point.z = aspect_ratio * 100.0f;
            position_pub.publish(point_msg);
            legacy_published = true;
        }

        ROS_INFO_THROTTLE(1.0,
            "Detected %s: confidence=%.3f center=(%d,%d) area_ratio=%.4f",
            box.Class.c_str(), box.probability, center_x, center_y,
            detection_msg.area_ratio);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_object_position");
    MoveObject move_object;
    ros::spin();
    return 0;
}
