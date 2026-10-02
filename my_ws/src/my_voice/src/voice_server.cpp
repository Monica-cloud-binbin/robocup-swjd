#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <std_srvs/Trigger.h>

sound_play::SoundClient* sound_client;

bool play1(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    sound_client->playWave("/home/cwkj/1.wav");
    res.success = true;
    return true;
}

bool play2(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    sound_client->playWave("/home/cwkj/2.wav");
    res.success = true;
    return true;
}

bool play3(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    sound_client->playWave("/home/cwkj/3.wav");
    res.success = true;
    return true;
}

// 播报“正在绕飞”
// 以后只需要更换 /home/cwkj/4.wav 即可修改这句语音内容
bool play4(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    sound_client->playWave("/home/cwkj/4.wav");
    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voice_server");
    ros::NodeHandle nh;
    
    sound_client = new sound_play::SoundClient();
    
    ros::ServiceServer s1 = nh.advertiseService("play1", play1);
    ros::ServiceServer s2 = nh.advertiseService("play2", play2);
    ros::ServiceServer s3 = nh.advertiseService("play3", play3);
    ros::ServiceServer s4 = nh.advertiseService("play4", play4);
    
    ros::spin();
    return 0;
}
