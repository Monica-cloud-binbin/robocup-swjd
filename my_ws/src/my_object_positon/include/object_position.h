#ifndef __OBJECT_DARKNET_H__ 
#define __OBJECT_DARKNET_H__

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <yolov8_ros_msgs/BoundingBoxes.h>

using namespace cv;
using namespace std;

class MoveObject
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_color;
    image_transport::Subscriber image_sub_depth;
    
    ros::Subscriber Object_sub;
    ros::Subscriber camera_info_sub_; // subscribe the topic, which pubbed by depth image
    
    ros::Publisher position_pub;

    ros::Subscriber local_pos_sub;

    sensor_msgs::CameraInfo camera_info;

    /* Mat depthImage, colorImage; */
    Mat colorImage;
    Mat depthImage = Mat::zeros(480, 640, CV_16UC1); // size of image
    Point mousepos = Point(0, 0); // mousepoint to be map

    ros::ServiceClient pose;
    tf::TransformListener listener;

    float real_x, real_y, real_z;
    bool detect_object, get_object_position; // running flag
    geometry_msgs::PoseStamped Object_pose, Object_pose_tmp;

public:
    MoveObject();
    ~MoveObject();
    bool move_finish;
    
    void ObjectCallback(const yolov8_ros_msgs::BoundingBoxes &object_tmp);
    void cameraInfoCb(const sensor_msgs::CameraInfo &msg);
    void imageDepthCb(const sensor_msgs::ImageConstPtr &msg);
    void imageColorCb(const sensor_msgs::ImageConstPtr &msg);
};

#endif
