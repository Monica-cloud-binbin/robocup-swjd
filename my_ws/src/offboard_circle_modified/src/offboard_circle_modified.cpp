
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
#include <yolov8_ros_msgs/BoundingBoxes.h>
#include <std_srvs/Trigger.h>

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

void object_position_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    bbox_ratio = msg->point.z/100;
    target_x = msg->point.x;
    last_target_time = ros::Time::now();
}
void depth_object_position_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    depth = msg->point.z;
    last_depth_time = ros::Time::now();
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


// ================================================================
// 黄色气球任务参数
// ================================================================
// 任务顺序：卫星绕飞 -> 飞到黄色气球搜索点/可调高度 -> 原地自转搜索
// -> 视觉逼近 -> 面积达到阈值停止 -> 返回原始起飞点 -> 降落。
std::string yellow_topic = "/yellow_balloon_position";
double yellow_balloon_altitude = 1.8;     // 黄气球搜索/逼近的实际local odom高度（米）
double yellow_balloon_x = 0.0;             // launch中填写的黄气球/搜索区域X
double yellow_balloon_y = 0.0;             // launch中填写的黄气球/搜索区域Y
double yellow_search_yaw_rate = 0.20;      // 搜索自转角速度 rad/s
double yellow_target_x = 320.0;             // 图像水平中心
double yellow_center_tolerance = 30.0;     // 水平像素容差
double yellow_yaw_kp = 0.002;              // 像素误差到yaw速度比例
double yellow_approach_speed = 0.25;        // 远距离逼近速度 m/s
double yellow_slow_approach_speed = 0.08;   // 接近时减速速度 m/s
double yellow_area_slow_threshold = 0.10;   // 达到后降低逼近速度
double yellow_area_contact_threshold = 0.25;// 达到后停止逼近
double yellow_message_timeout = 0.5;        // 视觉消息有效期
double yellow_loss_return_timeout = 1.0;    // 丢失视觉多久返回搜索
double yellow_search_timeout = 60.0;        // 搜索提示超时时间
int yellow_detect_count_required = 3;       // 连续新消息数

double yellow_target_pixel_x = 320.0;
double yellow_target_pixel_y = 240.0;
double yellow_area_ratio = 0.0;
ros::Time last_yellow_time;

// 消息序号：避免20Hz主循环重复计算同一条视觉消息
unsigned long long yellow_message_seq = 0;
unsigned long long yellow_last_counted_seq = 0;
int yellow_detect_count = 0;

void yellow_balloon_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (!std::isfinite(msg->point.x) ||
        !std::isfinite(msg->point.y) ||
        !std::isfinite(msg->point.z))
    {
        ROS_WARN_THROTTLE(1.0, "Invalid yellow balloon message: NaN/Inf");
        return;
    }

    yellow_target_pixel_x = msg->point.x;
    yellow_target_pixel_y = msg->point.y;

    // 面积比例统一限制在[0,1]
    yellow_area_ratio = msg->point.z;
    if (yellow_area_ratio < 0.0) yellow_area_ratio = 0.0;
    if (yellow_area_ratio > 1.0) yellow_area_ratio = 1.0;

    last_yellow_time = ros::Time::now();
    ++yellow_message_seq;
}

bool yellowMessageFresh()
{
    return !last_yellow_time.isZero() &&
           (ros::Time::now() - last_yellow_time <
            ros::Duration(yellow_message_timeout));
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

    private_nh.param<double>("aspect_threshold", aspect_threshold, 1.1);
    private_nh.param<double>("delta_omega", delta_omega, 0.05);

    private_nh.param<double>("R_circle_0", R_circle_0, 1.7);
    private_nh.param<double>("R_circle_1", R_circle_1, 1.0);
    private_nh.param<double>("R_circle_2", R_circle_2, 1.0);
    private_nh.param<double>("R_circle_3", R_circle_3, 1.0);
    private_nh.param<double>("R_circle_4", R_circle_4, 1.5);
    private_nh.param<double>("omega_base", omega_base, 0.137);

    private_nh.param<double>("depth_target", depth_target, 0.385);
    private_nh.param<double>("Kp_depth", Kp_depth, 1.0);

    // ============================================================
    // 黄色气球参数：全部可以直接在launch中修改
    // ============================================================
    private_nh.param<std::string>("yellow_topic", yellow_topic, "/yellow_balloon_position");
    private_nh.param<double>("yellow_balloon_altitude", yellow_balloon_altitude, 1.8);
    private_nh.param<double>("yellow_balloon_x", yellow_balloon_x, 0.0);
    private_nh.param<double>("yellow_balloon_y", yellow_balloon_y, 0.0);
    private_nh.param<double>("yellow_search_yaw_rate", yellow_search_yaw_rate, 0.20);
    private_nh.param<double>("yellow_target_x", yellow_target_x, 320.0);
    private_nh.param<double>("yellow_center_tolerance", yellow_center_tolerance, 30.0);
    private_nh.param<double>("yellow_yaw_kp", yellow_yaw_kp, 0.002);
    private_nh.param<double>("yellow_approach_speed", yellow_approach_speed, 0.25);
    private_nh.param<double>("yellow_slow_approach_speed", yellow_slow_approach_speed, 0.08);
    private_nh.param<double>("yellow_area_slow_threshold", yellow_area_slow_threshold, 0.10);
    private_nh.param<double>("yellow_area_contact_threshold", yellow_area_contact_threshold, 0.25);
    private_nh.param<double>("yellow_message_timeout", yellow_message_timeout, 0.5);
    private_nh.param<double>("yellow_loss_return_timeout", yellow_loss_return_timeout, 1.0);
    private_nh.param<double>("yellow_search_timeout", yellow_search_timeout, 60.0);
    private_nh.param<int>("yellow_detect_count_required", yellow_detect_count_required, 3);

    ROS_INFO("ALTITUDE: %f", ALTITUDE);
    ROS_INFO("Satellite center offset: x=%f, y=%f", position_x, position_y);
    ROS_INFO("Circle R: %.2f %.2f %.2f %.2f, omega=%.3f",
             R_circle_0, R_circle_1, R_circle_2, R_circle_3, omega_base);

    ROS_INFO("===== Yellow balloon parameters =====");
    ROS_INFO("yellow vision topic: %s", yellow_topic.c_str());
    ROS_INFO("search point: x=%.3f y=%.3f, altitude=%.3f",
             yellow_balloon_x, yellow_balloon_y, yellow_balloon_altitude);
    ROS_INFO("search yaw rate=%.3f rad/s", yellow_search_yaw_rate);
    ROS_INFO("approach speed=%.3f / %.3f m/s",
             yellow_approach_speed, yellow_slow_approach_speed);
    ROS_INFO("area slow threshold=%.3f", yellow_area_slow_threshold);
    ROS_INFO("area stop threshold=%.3f", yellow_area_contact_threshold);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_multi_position");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");  
    // 加载参数
    loadParameters(private_nh);

    last_target_time = ros::Time::now();
    ros::Subscriber object_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(
    "/object_position", 10, object_position_cb);
    ros::Subscriber depth_object_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(
    "/depth_object_position", 10, depth_object_position_cb);
    // 黄色气球视觉：PointStamped.x=像素X，y=像素Y，z=面积比例[0,1]
    ros::Subscriber yellow_balloon_sub = nh.subscribe<geometry_msgs::PointStamped>(
        yellow_topic, 10, yellow_balloon_cb);
    last_yellow_time = ros::Time(0);
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
    

    // ================================================================
    // 卫星绕飞结束后的黄色气球任务
    // task 0~3：原卫星任务；task 4：黄色气球任务
    // ================================================================
    {
        ros::Rate mission_rate(20.0);

        double cx = init_position_x_take_off + position_x;
        double cy = init_position_y_take_off + position_y;
        double theta = atan2(local_pos.pose.pose.position.y - cy,
                             local_pos.pose.pose.position.x - cx);

        int vision_count = 0;
        double R_circle = R_circle_0;
        double omega_circle = omega_base;
        int task_num = 0;
        int task_num_last = 13;
        float error_x = 0.0f;
        float last_error_x = 0.0f;

        enum YellowState {
            YELLOW_MOVE, YELLOW_SEARCH, YELLOW_APPROACH,
            YELLOW_HOLD, YELLOW_RETURN, YELLOW_LAND
        };
        YellowState yellow_state = YELLOW_MOVE;
        ros::Time yellow_state_start = ros::Time::now();
        ros::Time yellow_loss_start;
        double yellow_command_yaw = yaw;
        bool yellow_initialized = false;

        // launch中的黄色气球坐标作为“搜索区域”坐标，高度也完全由launch参数决定。
        // 真正识别后，逼近控制仍然完全依据视觉像素和面积。
        const double yellow_search_x = yellow_balloon_x;
        const double yellow_search_y = yellow_balloon_y;
        const double yellow_search_z = yellow_balloon_altitude;

        while (ros::ok())
        {
            ros::spinOnce();

            if (!flag_init_position) {
                mission_rate.sleep();
                continue;
            }

            const ros::Time now = ros::Time::now();

            // ========================================================
            // 1. 原来的卫星绕飞：task 0~3
            // ========================================================
            if (task_num >= 0 && task_num <= 3)
            {
                // task1/2/3各20秒，task3结束后进入task4
                if (task_num > 0 && task_num < 4 &&
                    now - task_start_time > ros::Duration(20.0))
                {
                    task_num++;
                    task_start_time = now;
                    ROS_INFO("Satellite circle segment finished, task=%d",
                             task_num);
                }

                if (task_num_last != task_num &&
                    now - task_start_time > ros::Duration(6.0))
                {
                    task_num_last = task_num;
                    if (task_num == 1) client1.call(trigger);
                    if (task_num == 2) client2.call(trigger);
                    if (task_num == 3) client3.call(trigger);
                    ROS_INFO("task now: %d", task_num);
                }

                switch (task_num)
                {
                    case 0:
                    {
                        if (first_is_target_detected())
                            vision_count++;
                        else
                            vision_count = 0;

                        if (vision_count > 5)
                        {
                            task_start_time = now;
                            omega_circle = omega_base;
                            R_circle = R_circle_1;
                            task_num = 1;
                            ROS_INFO("Satellite detected, start circle.");
                        }
                        else
                        {
                            omega_circle = -omega_base / 2.0;
                            R_circle = R_circle_0;
                        }
                        break;
                    }

                    case 1:
                        R_circle = R_circle_1;
                        break;
                    case 2:
                        R_circle = R_circle_2;
                        break;
                    case 3:
                        R_circle = R_circle_3;
                        break;
                }

                // 原来的卫星视觉跟踪角速度控制
                if (task_num > 0 && task_num < 4)
                {
                    if (is_target_detected())
                    {
                        error_x = target_x - x_center;
                        if (error_x > 200) error_x = 200;
                        if (error_x < -200) error_x = -200;

                        Kp = (error_x > 0) ? Kp_fast : Kp_slow;
                        float d_error_x =
                            (error_x - last_error_x) * 20.0f;
                        last_error_x = error_x;

                        if (bbox_ratio > aspect_threshold)
                        {
                            if (target_x < 320.0f)
                                omega_circle = omega_base +
                                    Kp * error_x + Kd * d_error_x -
                                    delta_omega;
                            else
                                omega_circle = omega_base +
                                    Kp * error_x + Kd * d_error_x +
                                    delta_omega;
                        }
                        else
                        {
                            omega_circle = omega_base +
                                Kp * error_x + Kd * d_error_x;
                        }
                    }
                    else
                    {
                        omega_circle = omega_base;
                        last_error_x = 0.0f;
                    }
                }

                if (task_num == 0)
                {
                    if (omega_circle > -0.02) omega_circle = -0.02;
                    if (omega_circle < -0.25) omega_circle = -0.25;
                }
                else
                {
                    if (omega_circle < 0.02) omega_circle = 0.02;
                    if (omega_circle > 0.25) omega_circle = 0.25;
                }

                theta += omega_circle / 20.0;

                double x_d = cx + R_circle * cos(theta);
                double y_d = cy + R_circle * sin(theta);
                double z_d = init_position_z_take_off + ALTITUDE;

                double yaw_face =
                    atan2(cy - local_pos.pose.pose.position.y,
                          cx - local_pos.pose.pose.position.x);

                tf::Quaternion q_tf;
                q_tf.setRPY(0, 0, yaw_face);
                tf::quaternionTFToMsg(q_tf, pose.pose.orientation);

                pose.header.stamp = now;
                pose.pose.position.x = x_d;
                pose.pose.position.y = y_d;
                pose.pose.position.z = z_d;

                ROS_INFO_THROTTLE(
                    2.0,
                    "SATELLITE: task=%d R=%.2f omega=%.3f",
                    task_num, R_circle, omega_circle);
            }

            // ========================================================
            // 2. 绕飞结束：黄色气球任务
            // ========================================================
            else if (task_num == 4)
            {
                if (!yellow_initialized)
                {
                    yellow_initialized = true;
                    yellow_state = YELLOW_MOVE;
                    yellow_state_start = now;
                    yellow_command_yaw = yaw;
                    yellow_detect_count = 0;
                    yellow_last_counted_seq = yellow_message_seq;
                    yellow_loss_start = ros::Time(0);

                    ROS_INFO("===== SATELLITE CIRCLE FINISHED =====");
                    ROS_INFO("Yellow search point=(%.3f, %.3f), height=%.3f",
                             yellow_search_x, yellow_search_y,
                             yellow_balloon_altitude);
                }

                // ----------------------------------------------------
                // 2.1 飞到黄色气球搜索点，高度1.8m
                // ----------------------------------------------------
                if (yellow_state == YELLOW_MOVE)
                {
                    pose.header.stamp = now;
                    pose.pose.position.x = yellow_search_x;
                    pose.pose.position.y = yellow_search_y;
                    pose.pose.position.z = yellow_search_z;

                    tf::Quaternion q_tf;
                    q_tf.setRPY(0, 0, yellow_command_yaw);
                    tf::quaternionTFToMsg(q_tf, pose.pose.orientation);

                    double dx = local_pos.pose.pose.position.x -
                                yellow_search_x;
                    double dy = local_pos.pose.pose.position.y -
                                yellow_search_y;
                    double dz = local_pos.pose.pose.position.z -
                                yellow_search_z;
                    double xy_error = sqrt(dx * dx + dy * dy);

                    if (xy_error < 0.20 && fabs(dz) < 0.20)
                    {
                        yellow_state = YELLOW_SEARCH;
                        yellow_state_start = now;
                        yellow_command_yaw = yaw;
                        yellow_detect_count = 0;
                        yellow_last_counted_seq = yellow_message_seq;

                        ROS_INFO("Reached yellow search point. "
                                 "Start 360-degree search.");
                    }
                }

                // ----------------------------------------------------
                // 2.2 原地自转识别
                // ----------------------------------------------------
                else if (yellow_state == YELLOW_SEARCH)
                {
                    if (yellowMessageFresh())
                    {
                        if (yellow_message_seq != yellow_last_counted_seq)
                        {
                            yellow_last_counted_seq = yellow_message_seq;
                            yellow_detect_count++;
                        }
                    }
                    else
                    {
                        yellow_detect_count = 0;
                    }

                    if (yellow_detect_count >=
                        yellow_detect_count_required)
                    {
                        yellow_state = YELLOW_APPROACH;
                        yellow_state_start = now;
                        yellow_loss_start = ros::Time(0);

                        ROS_INFO("Yellow balloon detected: "
                                 "pixel_x=%.1f area=%.3f",
                                 yellow_target_pixel_x,
                                 yellow_area_ratio);
                    }

                    // 超时只报警，不盲目飞行，继续原地搜索
                    if (now - yellow_state_start >
                        ros::Duration(yellow_search_timeout))
                    {
                        ROS_WARN_THROTTLE(
                            5.0,
                            "Yellow search timeout; continue rotating.");
                    }

                    // 原地自转：XY固定，只改变yaw
                    yellow_command_yaw += yellow_search_yaw_rate / 20.0;

                    pose.header.stamp = now;
                    pose.pose.position.x = yellow_search_x;
                    pose.pose.position.y = yellow_search_y;
                    pose.pose.position.z = yellow_search_z;

                    tf::Quaternion q_tf;
                    q_tf.setRPY(0, 0, yellow_command_yaw);
                    tf::quaternionTFToMsg(q_tf, pose.pose.orientation);
                }

                // ----------------------------------------------------
                // 2.3 识别后逼近
                // ----------------------------------------------------
                else if (yellow_state == YELLOW_APPROACH)
                {
                    // 视觉丢失：立即停止向前，超过1秒回搜索
                    if (!yellowMessageFresh())
                    {
                        if (yellow_loss_start.isZero())
                            yellow_loss_start = now;

                        pose.header.stamp = now;
                        pose.pose.position.x =
                            local_pos.pose.pose.position.x;
                        pose.pose.position.y =
                            local_pos.pose.pose.position.y;
                        pose.pose.position.z = yellow_search_z;

                        tf::Quaternion q_tf;
                        q_tf.setRPY(0, 0, yellow_command_yaw);
                        tf::quaternionTFToMsg(q_tf, pose.pose.orientation);

                        if (now - yellow_loss_start >
                            ros::Duration(yellow_loss_return_timeout))
                        {
                            ROS_WARN("Yellow vision lost; return to search.");
                            yellow_state = YELLOW_SEARCH;
                            yellow_state_start = now;
                            yellow_detect_count = 0;
                            yellow_last_counted_seq = yellow_message_seq;
                            yellow_loss_start = ros::Time(0);
                        }

                        local_pos_pub.publish(pose);
                        mission_rate.sleep();
                        continue;
                    }

                    yellow_loss_start = ros::Time(0);

                    // 面积达到阈值：停止逼近
                    if (yellow_area_ratio >=
                        yellow_area_contact_threshold)
                    {
                        ROS_INFO("Yellow target distance reached: "
                                 "area=%.3f >= %.3f",
                                 yellow_area_ratio,
                                 yellow_area_contact_threshold);

                        yellow_state = YELLOW_HOLD;
                        yellow_state_start = now;
                        yellow_command_yaw = yaw;
                    }
                    else
                    {
                        // 水平像素误差控制yaw
                        double pixel_error =
                            yellow_target_pixel_x - yellow_target_x;

                        double yaw_rate = 0.0;
                        if (fabs(pixel_error) >
                            yellow_center_tolerance)
                        {
                            yaw_rate = -yellow_yaw_kp * pixel_error;

                            if (yaw_rate > yellow_search_yaw_rate)
                                yaw_rate = yellow_search_yaw_rate;
                            if (yaw_rate < -yellow_search_yaw_rate)
                                yaw_rate = -yellow_search_yaw_rate;
                        }

                        yellow_command_yaw += yaw_rate / 20.0;

                        // 面积达到slow阈值后减速
                        double speed =
                            (yellow_area_ratio >=
                             yellow_area_slow_threshold)
                            ? yellow_slow_approach_speed
                            : yellow_approach_speed;

                        // 沿当前yaw方向向前逼近
                        pose.header.stamp = now;
                        pose.pose.position.x =
                            local_pos.pose.pose.position.x +
                            speed * cos(yellow_command_yaw) / 20.0;
                        pose.pose.position.y =
                            local_pos.pose.pose.position.y +
                            speed * sin(yellow_command_yaw) / 20.0;
                        pose.pose.position.z = yellow_search_z;

                        tf::Quaternion q_tf;
                        q_tf.setRPY(0, 0, yellow_command_yaw);
                        tf::quaternionTFToMsg(q_tf,
                                              pose.pose.orientation);

                        ROS_INFO_THROTTLE(
                            1.0,
                            "YELLOW_APPROACH: area=%.3f "
                            "pixel_error=%.1f speed=%.3f",
                            yellow_area_ratio,
                            pixel_error, speed);
                    }
                }

                // ----------------------------------------------------
                // 2.4 达到面积阈值：停止逼近
                // ----------------------------------------------------
                else if (yellow_state == YELLOW_HOLD)
                {
                    pose.header.stamp = now;
                    pose.pose.position.x =
                        local_pos.pose.pose.position.x;
                    pose.pose.position.y =
                        local_pos.pose.pose.position.y;
                    pose.pose.position.z = yellow_search_z;

                    tf::Quaternion q_tf;
                    q_tf.setRPY(0, 0, yellow_command_yaw);
                    tf::quaternionTFToMsg(q_tf,
                                          pose.pose.orientation);

                    // 保持1秒后返航
                    if (now - yellow_state_start >
                        ros::Duration(1.0))
                    {
                        yellow_state = YELLOW_RETURN;
                        ROS_INFO("Yellow approach finished. "
                                 "Return to original takeoff point.");
                    }
                }

                // ----------------------------------------------------
                // 2.5 返回最初起飞点
                // ----------------------------------------------------
                else if (yellow_state == YELLOW_RETURN)
                {
                    pose.header.stamp = now;
                    pose.pose.position.x = init_position_x_take_off;
                    pose.pose.position.y = init_position_y_take_off;
                    pose.pose.position.z =
                        yellow_search_z;

                    double return_yaw =
                        atan2(init_position_y_take_off -
                                  local_pos.pose.pose.position.y,
                              init_position_x_take_off -
                                  local_pos.pose.pose.position.x);

                    tf::Quaternion q_tf;
                    q_tf.setRPY(0, 0, return_yaw);
                    tf::quaternionTFToMsg(q_tf,
                                          pose.pose.orientation);

                    double dx =
                        local_pos.pose.pose.position.x -
                        init_position_x_take_off;
                    double dy =
                        local_pos.pose.pose.position.y -
                        init_position_y_take_off;
                    double return_distance = sqrt(dx * dx + dy * dy);

                    ROS_INFO_THROTTLE(
                        2.0,
                        "YELLOW_RETURN: distance=%.2f m",
                        return_distance);

                    if (return_distance < 0.20)
                    {
                        yellow_state = YELLOW_LAND;
                        yellow_state_start = now;
                        ROS_INFO("Returned to original point. "
                                 "Prepare AUTO.LAND.");
                    }
                }

                // ----------------------------------------------------
                // 2.6 起点保持3秒后降落
                // ----------------------------------------------------
                else if (yellow_state == YELLOW_LAND)
                {
                    pose.header.stamp = now;
                    pose.pose.position.x = init_position_x_take_off;
                    pose.pose.position.y = init_position_y_take_off;
                    pose.pose.position.z =
                        yellow_search_z;

                    tf::Quaternion q_tf;
                    q_tf.setRPY(0, 0, yellow_command_yaw);
                    tf::quaternionTFToMsg(q_tf,
                                          pose.pose.orientation);

                    if (now - yellow_state_start >
                        ros::Duration(3.0))
                    {
                        land_set_mode.request.custom_mode = "AUTO.LAND";

                        if (set_mode_client.call(land_set_mode) &&
                            land_set_mode.response.mode_sent)
                        {
                            ROS_INFO("AUTO.LAND enabled. "
                                     "Mission completed.");
                            break;
                        }
                    }
                }
            }

            local_pos_pub.publish(pose);
            mission_rate.sleep();
        }
    }

    return 0;
}
