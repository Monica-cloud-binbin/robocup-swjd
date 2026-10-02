
//包含ROS和MAVROS相关头文件 
#include <string> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>   
#include <std_srvs/Trigger.h>
#include <my_object_position/ObjectDetection.h>

int flag = 1;
double ALTITUDE = 0.825;
float target_x = 320; // 初始默认在中心
float x_center = 320;
float depth = 0;
mavros_msgs::State current_state;
mavros_msgs::SetMode land_set_mode;
void state_cb(const mavros_msgs::State::ConstPtr& msg);

//定义变量，用于接收无人机的里程计信息
tf::Quaternion quat; 
double roll, pitch, yaw;
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;

ros::Time last_target_time;
ros::Time last_depth_time;
ros::Time task_start_time;
ros::ServiceClient play4_client;
bool play4_started = false;

bool  flag_init_position = false;
nav_msgs::Odometry local_pos;

float bbox_ratio = 0.0f;      // 宽高比

double position_x = 2;
double position_y = -2;
double Kp_fast = 0.00035f;
double Kp_slow = 0.00020f;
double Kp = 0.0025; // 比例系数，可调
double Kd = 0.0003;
double aspect_threshold = 1.1f;  // 宽高比阈值，可调
double delta_omega = 0.05f;      // 固定增量角速度，正值表示增加角速度     2.8
double R_circle_0 = 1.7;
double R_circle_1 = 1;
double R_circle_2 = 1;
double R_circle_3 = 1;
double R_circle_4 = 1.5;
double omega_base = 0.137; // 基础角速度
double depth_target = 0.385; // 目标深度 (0.34~0.43) 中间值
double Kp_depth = 1; // 控制增益 (可调)

double yellow_balloon_x = 3.5;
double yellow_balloon_y = -1.0;
double yellow_balloon_altitude = 1.8;

// 卫星和黄气球共用 my_object_position 的置信度过滤与结构化检测接口。
bool yellow_detection_valid = false;
int yellow_x_pixel = 320;
int yellow_y_pixel = 240;
float yellow_area_ratio = 0.0f;
ros::Time last_yellow_detection_time;

double yellow_image_center_x = 320.0;
double yellow_image_center_y = 240.0;
double yellow_pixel_tolerance = 60.0;
double yellow_detection_timeout = 0.5;
std::string satellite_class_name;
std::string yellow_balloon_class_name;

enum YellowTaskState
{
    YELLOW_MOVE_TO_AREA = 0,
    YELLOW_WAIT_CAPTURE = 1,
    YELLOW_DESCEND = 2,
    YELLOW_RETURN_HOME = 3,
    YELLOW_LAND = 4
};

YellowTaskState yellow_task_state = YELLOW_MOVE_TO_AREA;
int yellow_capture_count = 0;

void object_detection_cb(
    const my_object_position::ObjectDetection::ConstPtr& msg)
{
    if (msg->class_name == satellite_class_name)
    {
        bbox_ratio = msg->aspect_ratio;
        target_x = msg->center_x;
        last_target_time = ros::Time::now();
    }
    else if (msg->class_name == yellow_balloon_class_name)
    {
        yellow_detection_valid = true;
        yellow_x_pixel = msg->center_x;
        yellow_y_pixel = msg->center_y;
        yellow_area_ratio = msg->area_ratio;
        last_yellow_detection_time = ros::Time::now();
    }
}
void depth_object_position_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    depth = msg->point.z;
    last_depth_time = ros::Time::now();
}

bool yellow_balloon_is_detected()
{
    return yellow_detection_valid &&
           (ros::Time::now() - last_yellow_detection_time <
            ros::Duration(yellow_detection_timeout));
}

bool yellow_balloon_is_centered()
{
    if (!yellow_balloon_is_detected())
        return false;

    return (std::fabs(yellow_x_pixel - yellow_image_center_x) <= yellow_pixel_tolerance &&
            std::fabs(yellow_y_pixel - yellow_image_center_y) <= yellow_pixel_tolerance);
}
bool is_target_detected()
{
    return (ros::Time::now() - last_target_time < ros::Duration(0.5));
}
bool first_is_target_detected()
{
    // 条件1：目标最近更新时间在0.5秒内
    // 条件2：目标x坐标在270~370之间
    return ((ros::Time::now() - last_target_time < ros::Duration(0.5)) && (target_x >= 245 && target_x <= 370));
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//回调函数接收无人机的里程计信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x_take_off = local_pos.pose.pose.position.x;
	    init_position_y_take_off = local_pos.pose.pose.position.y;
	    init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;		    
    }
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void loadParameters(ros::NodeHandle& private_nh)
{
    private_nh.param<double>("ALTITUDE", ALTITUDE, 0.825);
    private_nh.param<double>("position_x", position_x, 2.0);
    private_nh.param<double>("position_y", position_y, -2.0);
    private_nh.param<double>("Kp_fast", Kp_fast, 0.00035);
    private_nh.param<double>("Kp_slow", Kp_slow, 0.00020);
    private_nh.param<double>("Kp", Kp, 0.0025);
    private_nh.param<double>("Kd", Kd, 0.0003);

    // 目标检测参数
    private_nh.param<double>("aspect_threshold", aspect_threshold, 1.1);
    private_nh.param<double>("delta_omega", delta_omega, 0.05);

    // 圆形轨迹参数
    private_nh.param<double>("R_circle_0", R_circle_0, 1.7);
    private_nh.param<double>("R_circle_1", R_circle_1, 1);
    private_nh.param<double>("R_circle_2", R_circle_2, 1);
    private_nh.param<double>("R_circle_3", R_circle_3, 1);
    private_nh.param<double>("R_circle_4", R_circle_4, 1.5);
    private_nh.param<double>("omega_base", omega_base, 0.137);

    // 深度控制参数
    private_nh.param<double>("depth_target", depth_target, 0.385);
    private_nh.param<double>("Kp_depth", Kp_depth, 1.0);
    private_nh.param<double>("yellow_balloon_x", yellow_balloon_x, 3.5);
    private_nh.param<double>("yellow_balloon_y", yellow_balloon_y, -1.0);
    private_nh.param<double>("yellow_balloon_altitude", yellow_balloon_altitude, 1.8);
    private_nh.param<double>("yellow_image_center_x", yellow_image_center_x, 320.0);
    private_nh.param<double>("yellow_image_center_y", yellow_image_center_y, 240.0);
    private_nh.param<double>("yellow_pixel_tolerance", yellow_pixel_tolerance, 60.0);
    private_nh.param<double>("yellow_detection_timeout", yellow_detection_timeout, 0.5);
    private_nh.param<std::string>("satellite_class_name", satellite_class_name, "");
    private_nh.param<std::string>("yellow_balloon_class_name", yellow_balloon_class_name, "");

    ROS_INFO("ALTITUDE: %f", ALTITUDE);
    ROS_INFO("position_x: %f", position_x);
    ROS_INFO("position_y: %f", position_y);
    ROS_INFO("  Kp_fast: %.6f, Kp_slow: %.6f", Kp_fast, Kp_slow);
    ROS_INFO("  Kp: %.6f, Kd: %.6f", Kp, Kd);
    ROS_INFO("  aspect_threshold : %.1f,delta_omega: %.3f", aspect_threshold, delta_omega);
    ROS_INFO("  Radius: R0=%.1f, R1=%.1f, R2=%.1f, R3=%.1f, R4=%.1f", R_circle_0, R_circle_1, R_circle_2, R_circle_3, R_circle_4);
    ROS_INFO("  omega_base: %.3f", omega_base);
    ROS_INFO("  Target Depth: %.3f, Kp_depth: %.1f", depth_target, Kp_depth);
    ROS_INFO("  Yellow visual center: (%.1f, %.1f), tolerance: %.1f px, timeout: %.2f s",
             yellow_image_center_x, yellow_image_center_y,
             yellow_pixel_tolerance, yellow_detection_timeout);
    ROS_INFO("  YOLO classes: satellite='%s', yellow_balloon='%s'",
             satellite_class_name.c_str(), yellow_balloon_class_name.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_multi_position");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");  
    // 加载参数
    loadParameters(private_nh);

    if (satellite_class_name.empty() || yellow_balloon_class_name.empty() ||
        satellite_class_name == yellow_balloon_class_name)
    {
        ROS_FATAL("Set distinct, non-empty satellite_class_name and "
                  "yellow_balloon_class_name parameters before flight.");
        return 1;
    }

    last_target_time = ros::Time(0);
    last_yellow_detection_time = ros::Time(0);
    ros::Subscriber object_detection_sub =
        nh.subscribe<my_object_position::ObjectDetection>(
            "/object_detection", 10, object_detection_cb);
    ros::Subscriber depth_object_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(
    "/depth_object_position", 10, depth_object_position_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);

    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>(
        "mavros/local_position/odom", 10, local_pos_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");

    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Trigger>("play1");
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Trigger>("play2");
    ros::ServiceClient client3 = nh.serviceClient<std_srvs::Trigger>("play3");
    play4_client = nh.serviceClient<std_srvs::Trigger>("play4");
    std_srvs::Trigger trigger;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    /*/ 等待里程计初始化（有一个超时以防死等）
    ros::Time wait_start = ros::Time::now();
    while (ros::ok() && !flag_init_position)
    {
        ros::spinOnce();
        if (ros::Time::now()-wait_start > ros::Duration(2.5)) {
            ROS_WARN("waiting for local_pos... still not received. continuing to check.");
        }
        rate.sleep();
    }*/

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =init_position_x_take_off + 0;
    pose.pose.position.y =init_position_y_take_off + 0;
    pose.pose.position.z =init_position_z_take_off + ALTITUDE;
     
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

 	//此处满足一次请求进入offboard模式即可
    while(ros::ok())
    {
    	//请求进入OFFBOARD模式
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else 
		{
			//请求解锁
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		        }
		        	last_request = ros::Time::now();
			}
		}
	    
	    if(fabs(local_pos.pose.pose.position.z- init_position_z_take_off -ALTITUDE)<0.2)
		{	
			if(ros::Time::now() - last_request > ros::Duration(3.0))
			{
				break;
			}
		}
		//发布期望位置信息（过渡、保持）
		pose.pose.position.x =init_position_x_take_off + 0;
		pose.pose.position.y =init_position_y_take_off + 0;
		pose.pose.position.z =init_position_z_take_off + ALTITUDE;
		local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }   
    
    // ==== Minimal change: continuous circle trajectory ====
    {   
        ros::Time last_time = ros::Time::now();
        ros::Time return_time;
        
        double cx = init_position_x_take_off + position_x;
        double cy = init_position_y_take_off + position_y;
        double theta = 0.0;
        bool init_flag = true;
        bool return_flag = false;
        int vision_count = 0;
        double R_circle = R_circle_0;
        double R_circle_last = R_circle_3;
        double omega_circle = omega_base;
        int task_num = 0;
        int task_num_last = 13;
        float error_x = 0;
        float last_error_x = 0;
        
  

        if (init_flag) {
            theta = atan2(local_pos.pose.pose.position.y - cy,
                          local_pos.pose.pose.position.x - cx);
            init_flag = false;
        }

        ros::Rate circle_rate(20.0);
        while (ros::ok())
        {
            if (!flag_init_position) {
                circle_rate.sleep();
                continue;
            }

            // ===== 任务切换 =====
            // task 0：原来的卫星搜索
            // task 1：识别卫星后，原来的 R1 绕飞轨迹；到达 R1 后调用 play4
            // task 2：play4 成功后的正式10秒绕飞（仍使用原来的圆轨迹控制）
            // task 4：黄气球任务
            if (task_num == 1 && !play4_started)
            {
                double current_radius = std::sqrt(
                    std::pow(local_pos.pose.pose.position.x - cx, 2) +
                    std::pow(local_pos.pose.pose.position.y - cy, 2));

                if (std::fabs(current_radius - R_circle_1) < 0.10)
                {
                    std_srvs::Trigger play4_srv;
                    if (play4_client.call(play4_srv) && play4_srv.response.success)
                    {
                        play4_started = true;
                        task_start_time = ros::Time::now();
                        task_num = 2;
                        ROS_INFO("play4 service succeeded. Start 10-second satellite orbit.");
                    }
                    else
                    {
                        ROS_WARN_THROTTLE(1.0, "play4 service failed. Waiting at R1 orbit radius.");
                    }
                }
            }

            if (task_num == 2 && ros::Time::now() - task_start_time > ros::Duration(10.0))
            {
                task_num = 4;
                task_start_time = ros::Time::now();
                yellow_task_state = YELLOW_MOVE_TO_AREA;
                yellow_capture_count = 0;
                ROS_INFO("Satellite 10-second orbit finished. Start yellow balloon mission.");
            }

            // ==== control ==== //
            {
                switch(task_num)
                {
                    case 0:
                    {
                        if (first_is_target_detected())
                        {
                            ROS_INFO("vision_count = %d",vision_count);
                            vision_count++;
                        }
                        else
                        {
                            vision_count = 0;
                        }

                        if(vision_count>5)
                        {
                            // 保持原来的逻辑：识别后进入 R1 圆轨迹。
                            // 语音不在这里直接播放，而是在真正到达 R1 后调用 play4。
                            omega_circle = omega_base;
                            R_circle = R_circle_1;
                            task_num = 1;
                            play4_started = false;
                        }
                        else
                        {
                            omega_circle = -omega_base/2;
                            R_circle = R_circle_0;
                        }
                        break;
                    }

                    case 1:
                    case 2:
                    {
                        // 保留原来的卫星绕飞半径逻辑。
                        // task 1 负责到达 R1；task 2 在 play4 成功后继续绕飞10秒。
                        R_circle = R_circle_1;
                        break;
                    }

                    case 4:
                    {
                        // 黄气球：先上升到指定高度，再水平飞到相对起飞点的指定位置。
                        R_circle = R_circle_4;
                        omega_circle = 0.0;
                        break;
                    }
                }

                // 原来的视觉跟踪角速度控制，只用于卫星绕飞 task 1/2。
                if(task_num == 1 || task_num == 2)
                {
                    if (is_target_detected())
                    {
                        error_x = target_x - x_center;
                        if (error_x > 200) error_x = 200;
                        if (error_x < -200) error_x = -200;

                        if(error_x > 0)
                            Kp = Kp_fast;
                        else
                            Kp = Kp_slow;

                        float d_error_x = (error_x - last_error_x) * 20.0;
                        last_error_x = error_x;

                        if (bbox_ratio > aspect_threshold)
                        {
                            if (target_x < 320.0f)
                                omega_circle = omega_base + Kp * error_x + Kd * d_error_x - delta_omega;
                            else
                                omega_circle = omega_base + Kp * error_x + Kd * d_error_x + delta_omega;
                        }
                        else
                            omega_circle = omega_base + Kp * error_x + Kd * d_error_x;
                    }
                    else
                    {
                        omega_circle = omega_base;
                        last_error_x = 0.0f;
                    }
                }
            }

            // ===== 黄气球任务：到指定区域后，用统一 YOLO 结果确认并完成捕获 =====
            // my_object_position 已完成置信度过滤并保留类别、中心和面积比例。
            // 这里不根据像素直接推算飞行方向；
            // 先按 launch 的坐标飞到黄气球区域，再用视觉确认“看到且居中”。
            if (task_num == 4)
            {
                const double yellow_x =
                    init_position_x_take_off + yellow_balloon_x;
                const double yellow_y =
                    init_position_y_take_off + yellow_balloon_y;
                const double yellow_z = yellow_balloon_altitude;
                const double return_z =
                    init_position_z_take_off + ALTITUDE;

                if (yellow_task_state == YELLOW_MOVE_TO_AREA)
                {
                    // 1. 先垂直上升到黄气球任务高度
                    if (std::fabs(local_pos.pose.pose.position.z - yellow_z) > 0.08)
                    {
                        pose.pose.position.x = local_pos.pose.pose.position.x;
                        pose.pose.position.y = local_pos.pose.pose.position.y;
                        pose.pose.position.z = yellow_z;
                    }
                    else
                    {
                        // 2. 再水平飞到 launch 指定的黄气球区域
                        double dx = yellow_x - local_pos.pose.pose.position.x;
                        double dy = yellow_y - local_pos.pose.pose.position.y;
                        double dxy = std::sqrt(dx * dx + dy * dy);

                        pose.pose.position.x = yellow_x;
                        pose.pose.position.y = yellow_y;
                        pose.pose.position.z = yellow_z;

                        if (dxy <= 0.12)
                        {
                            yellow_task_state = YELLOW_WAIT_CAPTURE;
                            yellow_capture_count = 0;
                            ROS_INFO("Reached yellow balloon search position. Start visual capture confirmation.");
                        }
                    }
                }
                else if (yellow_task_state == YELLOW_WAIT_CAPTURE)
                {
                    // 3. 到达指定区域后必须有视觉检测
                    // 4. 检测目标还需要进入图像中心附近
                    pose.pose.position.x = yellow_x;
                    pose.pose.position.y = yellow_y;
                    pose.pose.position.z = yellow_z;

                    if (yellow_balloon_is_centered())
                    {
                        yellow_capture_count++;

                        if (yellow_capture_count >= 5)
                        {
                            ROS_INFO(
                                "Yellow balloon captured: visual detection confirmed, "
                                "pixel=(%d,%d), area_ratio=%.4f.",
                                yellow_x_pixel, yellow_y_pixel, yellow_area_ratio);

                            yellow_task_state = YELLOW_DESCEND;
                            yellow_capture_count = 0;
                            task_start_time = ros::Time::now();
                        }
                    }
                    else
                    {
                        yellow_capture_count = 0;

                        if (yellow_balloon_is_detected())
                        {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "Yellow balloon detected but not centered: pixel=(%d,%d). Holding position.",
                                yellow_x_pixel, yellow_y_pixel);
                        }
                        else
                        {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "Waiting for valid yellow balloon detection. Holding position.");
                        }
                    }
                }
                else if (yellow_task_state == YELLOW_DESCEND)
                {
                    // 5. 捕获成功后，下降到与卫星任务相同的高度
                    pose.pose.position.x = yellow_x;
                    pose.pose.position.y = yellow_y;
                    pose.pose.position.z = return_z;

                    if (std::fabs(local_pos.pose.pose.position.z - return_z) <= 0.08)
                    {
                        yellow_task_state = YELLOW_RETURN_HOME;
                        ROS_INFO("Yellow balloon capture height reached. Returning home.");
                    }
                }
                else if (yellow_task_state == YELLOW_RETURN_HOME)
                {
                    // 6. 保持卫星任务高度，水平返回最初起飞点
                    pose.pose.position.x = init_position_x_take_off;
                    pose.pose.position.y = init_position_y_take_off;
                    pose.pose.position.z = return_z;

                    double home_dx =
                        init_position_x_take_off - local_pos.pose.pose.position.x;
                    double home_dy =
                        init_position_y_take_off - local_pos.pose.pose.position.y;
                    double home_dxy =
                        std::sqrt(home_dx * home_dx + home_dy * home_dy);

                    if (home_dxy <= 0.12)
                    {
                        yellow_task_state = YELLOW_LAND;
                        task_start_time = ros::Time::now();
                        ROS_INFO("Returned to takeoff point. Holding before AUTO.LAND.");
                    }
                }
                else if (yellow_task_state == YELLOW_LAND)
                {
                    // 7. 回到起飞点保持2秒，然后 AUTO.LAND
                    pose.pose.position.x = init_position_x_take_off;
                    pose.pose.position.y = init_position_y_take_off;
                    pose.pose.position.z = return_z;

                    if (ros::Time::now() - task_start_time > ros::Duration(2.0))
                    {
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        if (set_mode_client.call(land_set_mode) &&
                            land_set_mode.response.mode_sent)
                        {
                            ROS_INFO("AUTO.LAND enabled after yellow balloon mission.");
                            task_num = 5;
                        }
                        else
                        {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "Failed to enable AUTO.LAND. Holding at home position.");
                        }
                    }
                }

                pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(pose);
                ros::spinOnce();
                circle_rate.sleep();
                continue;
            }

            // 已经发送 AUTO.LAND 后，不再继续发送圆轨迹点。
            if (task_num == 5)
            {
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = init_position_x_take_off;
                pose.pose.position.y = init_position_y_take_off;
                pose.pose.position.z = init_position_z_take_off + ALTITUDE;
                local_pos_pub.publish(pose);
                ros::spinOnce();
                circle_rate.sleep();
                continue;
            }

            // 限制角速度
            if(task_num == 0)
            {
                if (omega_circle > -0.02) omega_circle = -0.02;
                if (omega_circle < -0.25) omega_circle = -0.25;
            }
            else
            {
                if (omega_circle < 0.02) omega_circle = 0.02;
                if (omega_circle > 0.25) omega_circle = 0.25;
            }

            theta += omega_circle * (1.0 / 20.0);
            
            

            //计算位置
            double x_d = cx + R_circle * cos(theta);
            double y_d = cy + R_circle * sin(theta);
            double z_d = init_position_z_take_off + ALTITUDE;

            //姿态计算
            double yaw_face = atan2(cy - local_pos.pose.pose.position.y , cx - local_pos.pose.pose.position.x);
            tf::Quaternion q_tf;
            q_tf.setRPY(0, 0, yaw_face);
            geometry_msgs::Quaternion q_msg;
            tf::quaternionTFToMsg(q_tf, q_msg);
            
            if(!return_flag)
            {
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = x_d;
                pose.pose.position.y = y_d;
                pose.pose.position.z = z_d;
                pose.pose.orientation = q_msg;
            }
            else
            {
                pose.header.stamp = ros::Time::now();
                if(ros::Time::now() - return_time > ros::Duration(15.0))
                {
                    land_set_mode.request.custom_mode = "AUTO.LAND";
                    if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
                    {
                        ROS_INFO("LAND mode enabled");
                    }
                }
                else
                {
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position.x = init_position_x_take_off - 0.05;
                    pose.pose.position.y = init_position_y_take_off + 0.05;
                    pose.pose.position.z = init_position_z_take_off + ALTITUDE;
                }
            }

		//打印信息
            bool detected = is_target_detected();
            ROS_INFO("Target detected: %s", detected ? "true" : "false");
            ROS_INFO("omega=%f,R=%f",omega_circle,R_circle);
            ROS_INFO("current_x=%f,current_y=%f,current_z = %f",local_pos.pose.pose.position.x,local_pos.pose.pose.position.y,local_pos.pose.pose.position.z);
            ROS_INFO("expect_x=%f,expect_y=%f",x_d,y_d);
            ROS_INFO("error_x=%f",error_x);
            ROS_INFO("task_num=%d",task_num);
            ROS_INFO("depth=%f",depth);
            ROS_INFO("bbox_ratio=%f",bbox_ratio);
            
            
            ros::spinOnce();
            local_pos_pub.publish(pose);
            circle_rate.sleep();
        }
    }

    ros::spinOnce();
    rate.sleep();
    return 0;
}
