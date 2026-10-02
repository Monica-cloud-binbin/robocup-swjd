#include "object_position.h"

bool temp_detect_object = false;

MoveObject::MoveObject() : it_(nh_), detect_object(false)
{
    position_pub = nh_.advertise<geometry_msgs::PointStamped>("/depth_object_position", 20);
    Object_sub = nh_.subscribe("/yolov8/camera_2/BoundingBoxes", 1, &MoveObject::ObjectCallback, this);
    image_sub_depth = it_.subscribe("/d435/aligned_depth_to_color/image_raw", 1, &MoveObject::imageDepthCb, this);
    image_sub_color = it_.subscribe("/d435/color/image_raw", 1, &MoveObject::imageColorCb, this);
    camera_info_sub_ = nh_.subscribe("/d435/depth/camera_info", 1, &MoveObject::cameraInfoCb, this);
}

MoveObject::~MoveObject()
{
    ROS_INFO("delete the class");
}

void MoveObject::cameraInfoCb(const sensor_msgs::CameraInfo &msg)
{
    camera_info = msg;
}

void MoveObject::imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        depthImage = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

string frame_id = "no object";
int temp = 0;

void MoveObject::ObjectCallback(const yolov8_ros_msgs::BoundingBoxes &object_tmp)
{
    if(object_tmp.bounding_boxes.empty())
    {
        ROS_WARN("No boxes recireved");
        detect_object = false;
        return;
    }
    string Object_class;
    string object_name;

    Object_class = object_tmp.bounding_boxes[0].Class;
    ros::param::get("object_name", object_name);

    detect_object = true;
    temp_detect_object = detect_object;
    frame_id = object_tmp.bounding_boxes[0].Class;
    mousepos.x = (object_tmp.bounding_boxes[0].xmin + object_tmp.bounding_boxes[0].xmax) / 2;
    mousepos.y = (object_tmp.bounding_boxes[0].ymin + object_tmp.bounding_boxes[0].ymax) / 2;
    if (mousepos.x != 0)
    {
        detect_object = true;
        temp_detect_object = detect_object;
    }
}

int count_no_object = 0;

void MoveObject::imageColorCb(const sensor_msgs::ImageConstPtr &msg)
{
    if (detect_object)
    {
        detect_object = false;
        count_no_object = 0;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            colorImage = cv_ptr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
        real_z = 0.001 * depthImage.at<u_int16_t>(mousepos.y, mousepos.x);
        real_x = (mousepos.x - camera_info.K.at(2)) / camera_info.K.at(0) * real_z;
        real_y = (mousepos.y - camera_info.K.at(5)) / camera_info.K.at(4) * real_z;
        if (real_x != 0 && real_y != 0)
        {
            get_object_position = true;
            Object_pose_tmp.header.frame_id = frame_id;
            Object_pose_tmp.header.stamp = ros::Time::now();
            Object_pose_tmp.pose.position.x = real_x;
            Object_pose_tmp.pose.position.y = real_y;
            Object_pose_tmp.pose.position.z = real_z;
            Object_pose_tmp.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            position_pub.publish(Object_pose_tmp);
        }
    }
    else
    {

        count_no_object++;
        if (count_no_object >= 5)
        {
            count_no_object = 0;
            get_object_position = false;
            Object_pose_tmp.header.frame_id = "camera_color_optical_frame";
            Object_pose_tmp.header.stamp = ros::Time::now();
            Object_pose_tmp.pose.position.x = 0;
            Object_pose_tmp.pose.position.y = 0;
            Object_pose_tmp.pose.position.z = 0;
            Object_pose_tmp.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            position_pub.publish(Object_pose_tmp);
            ROS_INFO("No object detected");
        }
        else
        {
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_object_position");
    MoveObject moveobject;
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
