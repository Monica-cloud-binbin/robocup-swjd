#include <ros/ros.h>
#include <std_srvs/Trigger.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_client");
    ros::NodeHandle nh;
    
    // 初始化客户端
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Trigger>("play1");
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Trigger>("play2");
    ros::ServiceClient client3 = nh.serviceClient<std_srvs::Trigger>("play3");
    
    std_srvs::Trigger trigger;
    
    // 播放语音1
    client1.call(trigger);
    ros::Duration(2).sleep();
    
    // 播放语音2
    client2.call(trigger);
    ros::Duration(2).sleep();
    
    // 播放语音3
    client3.call(trigger);
    
    return 0;
}
