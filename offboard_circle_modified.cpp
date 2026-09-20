
//包含ROS和MAVROS相关头文件 
#include <string> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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

    ROS_INFO("ALTITUDE: %f", ALTITUDE);
    ROS_INFO("position_x: %f", position_x);
    ROS_INFO("position_y: %f", position_y);
    ROS_INFO("  Kp_fast: %.6f, Kp_slow: %.6f", Kp_fast, Kp_slow);
    ROS_INFO("  Kp: %.6f, Kd: %.6f", Kp, Kd);
    ROS_INFO("  aspect_threshold : %.1f,delta_omega: %.3f", aspect_threshold, delta_omega);
    ROS_INFO("  Radius: R0=%.1f, R1=%.1f, R2=%.1f, R3=%.1f, R4=%.1f", R_circle_0, R_circle_1, R_circle_2, R_circle_3, R_circle_4);
    ROS_INFO("  omega_base: %.3f", omega_base);
    ROS_INFO("  Target Depth: %.3f, Kp_depth: %.1f", depth_target, Kp_depth);
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

            // ===== 任务计时切换 =====
            if(task_num>0 && task_num<4)
            {
                if (ros::Time::now() - task_start_time > ros::Duration(20.0))  // 每15秒切换一次任务
                {
                    task_num++;
                    task_start_time = ros::Time::now();       // 重新计时
                }         
            }   

            if(task_num_last != task_num && ros::Time::now() - task_start_time > ros::Duration(6.0))
            {
                task_num_last = task_num;
                if(task_num == 1)
                    client1.call(trigger);
                if(task_num == 2)
                    client2.call(trigger);
                if(task_num == 3)
                    client3.call(trigger);
                ROS_INFO("task now:%d",task_num);
            }
            
            // ==== control ==== //
            {
            	switch(task_num)
            	{
            	    case 0:
            	    {
            	    	if (first_is_target_detected())           //判断是否识别到
                        {
                            ROS_INFO("vision_count = %d",vision_count);
                            vision_count++;
                        }
            	    	else
                        {
                            vision_count = 0;
                        }
                        if(vision_count>5)                 //成功识别到5次，跳转下个任务点
                        {
                            task_start_time = ros::Time::now();  //开始绕飞
                            omega_circle = omega_base;
                            R_circle = R_circle_1;
                            task_num = 1;
                        }
                        else
                        {
                            omega_circle = -omega_base/2;
                            R_circle = R_circle_0;
                            
                        }
            	    	break;
            	    }
            	    case 1:
            	    {
                        R_circle = R_circle_1;
            	    	break;
            	    }
                    case 2:
            	    {
                        R_circle = R_circle_2;
            	    	break;
            	    }
                    case 3:
            	    {/*
                        // 仅当深度数据是最近0.5秒内有效时才进行修正
                        if (ros::Time::now() - last_depth_time < ros::Duration(0.2) && depth != 0)
                        {
                        	ROS_INFO("depth_start");
                            double depth_error = depth_target - depth; // 正值表示目标更远
                            double R_new = R_circle_3 + Kp_depth * depth_error;

                            // 限制R范围，防止过度修正
                            if (R_new < 0.6) R_new = 0.6;
                            if (R_new > 1.4) R_new = 1.4;

                            // 低通滤波平滑变化，防止抖动
                            R_circle = 0.8 * R_circle + 0.2 * R_new;
                            R_circle_last = R_circle;
                        }
                        else
                            R_circle = R_circle_last;
                            */
                            R_circle = R_circle_3;
            	    	break;
            	    }
                    case 4:
            	    {
                        R_circle = R_circle_4;
                        if(local_pos.pose.pose.position.x < cx && local_pos.pose.pose.position.y > cy && return_flag == false)  //开始返回
                        {
                            return_flag = true;
                            return_time = ros::Time::now();
                        }
            	    	break;
            	    }
            	}
                
                if(task_num>0 && task_num<4)
                {
                    // 偏差计算
                    if (is_target_detected())
                    {
                        // 目标存在：比例控制调整角速度
                        error_x = target_x - x_center; // 正值：目标偏右
                        
                        // 限制误差范围，防止极端值造成控制器“暴走”
                        if (error_x > 200) error_x = 200;
                        if (error_x < -200) error_x = -200;
                        
                        if(error_x > 0)
                            Kp = Kp_fast;
                        else
                            Kp = Kp_slow;

                        float d_error_x = (error_x - last_error_x) * 20.0;  // 因为频率20Hz
                        last_error_x = error_x;
                        /*omega_circle = omega_base + Kp * error_x + Kd * d_error_x;

                        // 判断是否为侧面
                        if (bbox_ratio > aspect_threshold)
                        {
                            if (target_x < 320.0f) 
                            {
                                // 目标左侧→减少角速度
                                omega_circle -= delta_omega;
                            } 
                            else
                            {
                                // 目标右侧→增加角速度
                                omega_circle += delta_omega;
                            }
                        }*/
                  
                        // 判断是否为侧面
                        if (bbox_ratio > aspect_threshold)
                        {
                            if (target_x < 320.0f) 
                            {
                                // 目标左侧→减少角速度
                                omega_circle = omega_base + Kp * error_x + Kd * d_error_x - delta_omega;
                            } 
                            else
                            {
                                // 目标右侧→增加角速度
                                omega_circle = omega_base + Kp * error_x + Kd * d_error_x + delta_omega;
                            }
                        }
                        else
                            omega_circle = omega_base + Kp * error_x + Kd * d_error_x;

                    }
                    else
                    {
                    // 没有目标：恢复基础速度，并清除误差累积
                    omega_circle = omega_base;
                    last_error_x = 0.0f;
                    }
                }
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
