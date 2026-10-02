#include <cmath>
#include <cstdint>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <my_object_position/ObjectDetection.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>

#include <offboard_circle_modified/avoidance_task.h>
#include <offboard_circle_modified/mission_types.h>
#include <offboard_circle_modified/satellite_task.h>
#include <offboard_circle_modified/yellow_balloon_task.h>

namespace
{

using offboard_test::AvoidanceConfig;
using offboard_test::AvoidanceTask;
using offboard_test::DesiredSetpoint;
using offboard_test::DetectionSnapshot;
using offboard_test::SatelliteConfig;
using offboard_test::SatelliteTask;
using offboard_test::TaskStatus;
using offboard_test::TaskUpdate;
using offboard_test::VehicleState;
using offboard_test::YellowBalloonConfig;
using offboard_test::YellowBalloonTask;

double ALTITUDE = 0.825;
mavros_msgs::State current_state;
tf::Quaternion quat;
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
bool flag_init_position = false;
nav_msgs::Odometry local_pos;

std::string satellite_class_name;
std::string yellow_balloon_class_name;
DetectionSnapshot satellite_detection;
DetectionSnapshot yellow_detection;
std::uint64_t satellite_detection_sequence = 0;
std::uint64_t yellow_detection_sequence = 0;

struct BaseConfig
{
    BaseConfig()
        : mission_mode(1), auto_land_after_mission(false),
          test_goal_x(4.3), test_goal_y(3.9),
          yellow_balloon_x(4.3), yellow_balloon_y(3.9),
          yellow_balloon_altitude(1.2),
          satellite_x(4.0), satellite_y(0.0)
    {
    }

    int mission_mode;
    bool auto_land_after_mission;
    double test_goal_x;
    double test_goal_y;
    double yellow_balloon_x;
    double yellow_balloon_y;
    double yellow_balloon_altitude;
    double satellite_x;
    double satellite_y;
    AvoidanceConfig avoidance;
    SatelliteConfig satellite;
    YellowBalloonConfig yellow;
};

BaseConfig config;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

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

void object_detection_cb(
    const my_object_position::ObjectDetection::ConstPtr& msg)
{
    DetectionSnapshot* snapshot = NULL;
    std::uint64_t* sequence = NULL;
    if (!satellite_class_name.empty() &&
        msg->class_name == satellite_class_name)
    {
        snapshot = &satellite_detection;
        sequence = &satellite_detection_sequence;
    }
    else if (!yellow_balloon_class_name.empty() &&
             msg->class_name == yellow_balloon_class_name)
    {
        snapshot = &yellow_detection;
        sequence = &yellow_detection_sequence;
    }
    else
    {
        return;
    }

    snapshot->class_name = msg->class_name;
    snapshot->sequence = ++(*sequence);
    snapshot->message_time = ros::Time::now();
    snapshot->confidence = msg->confidence;
    snapshot->center_x = msg->center_x;
    snapshot->center_y = msg->center_y;
    snapshot->aspect_ratio = msg->aspect_ratio;
    snapshot->area_ratio = msg->area_ratio;
    snapshot->valid = true;
}

void loadParameters(ros::NodeHandle& private_nh)
{
    private_nh.param<double>("ALTITUDE", ALTITUDE, 0.825);
    private_nh.param<int>("mission_mode", config.mission_mode, 1);
    private_nh.param<bool>("auto_land_after_mission",
                           config.auto_land_after_mission, false);
    private_nh.param<std::string>("satellite_class_name",
                                  satellite_class_name, "");
    private_nh.param<std::string>("yellow_balloon_class_name",
                                  yellow_balloon_class_name, "");

    private_nh.param<double>("test_goal_x", config.test_goal_x, 4.3);
    private_nh.param<double>("test_goal_y", config.test_goal_y, 3.9);
    private_nh.param<double>("yellow_balloon_x",
                             config.yellow_balloon_x, 4.3);
    private_nh.param<double>("yellow_balloon_y",
                             config.yellow_balloon_y, 3.9);
    private_nh.param<double>("yellow_balloon_altitude",
                             config.yellow_balloon_altitude, 1.2);
    private_nh.param<double>("satellite_x", config.satellite_x, 4.0);
    private_nh.param<double>("satellite_y", config.satellite_y, 0.0);

    private_nh.param<double>("obstacle_x", config.avoidance.obstacle_x, 1.8);
    private_nh.param<double>("obstacle_y", config.avoidance.obstacle_y, 3.9);
    private_nh.param<double>("obstacle_safe_distance",
                             config.avoidance.safety_radius, 1.0);
    private_nh.param<double>("avoidance_orbit_angle_deg",
                             config.avoidance.orbit_angle_deg, 180.0);
    private_nh.param<double>("avoidance_orbit_angular_speed",
                             config.avoidance.orbit_angular_speed, 0.10);
    private_nh.param<int>("avoidance_orbit_direction",
                          config.avoidance.orbit_direction, 1);
    private_nh.param<double>("arrive_tolerance",
                             config.avoidance.arrive_tolerance, 0.12);
    private_nh.param<double>("orbit_tracking_tolerance",
                             config.avoidance.orbit_tracking_tolerance, 0.25);

    private_nh.param<double>("satellite_image_center_x",
                             config.satellite.image_center_x, 320.0);
    private_nh.param<double>("satellite_kp_fast",
                             config.satellite.kp_fast, 0.00035);
    private_nh.param<double>("satellite_kp_slow",
                             config.satellite.kp_slow, 0.00020);
    private_nh.param<double>("satellite_kd",
                             config.satellite.kd, 0.0003);
    private_nh.param<double>("satellite_aspect_threshold",
                             config.satellite.aspect_threshold, 1.1);
    private_nh.param<double>("satellite_delta_omega",
                             config.satellite.delta_omega, 0.05);
    private_nh.param<double>("satellite_search_radius",
                             config.satellite.search_radius, 1.4);
    private_nh.param<double>("satellite_orbit_radius",
                             config.satellite.orbit_radius, 0.8);
    private_nh.param<double>("satellite_omega_base",
                             config.satellite.omega_base, 0.116);
    private_nh.param<double>("satellite_orbit_radius_tolerance",
                             config.satellite.orbit_radius_tolerance, 0.10);
    private_nh.param<double>("satellite_orbit_duration",
                             config.satellite.orbit_duration, 10.0);
    private_nh.param<double>("satellite_detection_timeout",
                             config.satellite.detection_timeout, 0.5);
    private_nh.param<int>("satellite_detection_count_required",
                          config.satellite.detection_count_required, 6);

    private_nh.param<double>("yellow_image_center_x",
                             config.yellow.image_center_x, 320.0);
    private_nh.param<double>("yellow_image_center_y",
                             config.yellow.image_center_y, 240.0);
    private_nh.param<double>("yellow_pixel_tolerance",
                             config.yellow.pixel_tolerance, 60.0);
    private_nh.param<double>("yellow_detection_timeout",
                             config.yellow.detection_timeout, 0.5);
    private_nh.param<int>("yellow_detection_count_required",
                          config.yellow.detection_count_required, 5);
}

bool validatePreflightParameters(std::string& error)
{
    if (config.mission_mode < 1 || config.mission_mode > 3)
    {
        error = "mission_mode must be 1, 2 or 3";
        return false;
    }
    if (ALTITUDE <= 0.0 || config.yellow_balloon_altitude <= 0.0)
    {
        error = "ALTITUDE and yellow_balloon_altitude must be positive offsets";
        return false;
    }
    if (config.mission_mode == 2 && yellow_balloon_class_name.empty())
    {
        error = "mission_mode=2 requires yellow_balloon_class_name";
        return false;
    }
    if (config.mission_mode == 3 &&
        (satellite_class_name.empty() || yellow_balloon_class_name.empty() ||
         satellite_class_name == yellow_balloon_class_name))
    {
        error = "mission_mode=3 requires distinct non-empty satellite and yellow class names";
        return false;
    }
    return true;
}

VehicleState currentVehicleState()
{
    VehicleState vehicle;
    vehicle.x = local_pos.pose.pose.position.x;
    vehicle.y = local_pos.pose.pose.position.y;
    vehicle.z = local_pos.pose.pose.position.z;
    vehicle.yaw = yaw;
    vehicle.odometry_time = local_pos.header.stamp;
    vehicle.valid = flag_init_position;
    return vehicle;
}

void applySetpoint(const DesiredSetpoint& desired,
                   geometry_msgs::PoseStamped& pose,
                   const ros::Time& stamp)
{
    pose.header.stamp = stamp;
    pose.pose.position.x = desired.x;
    pose.pose.position.y = desired.y;
    pose.pose.position.z = desired.z;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, desired.yaw);
    tf::quaternionTFToMsg(q, pose.pose.orientation);
}

}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_test_base");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    // 加载参数
    loadParameters(private_nh);

    std::string error;
    if (!validatePreflightParameters(error))
    {
        ROS_FATAL("Preflight parameter validation failed: %s", error.c_str());
        return 1;
    }

    ros::Subscriber object_detection_sub =
        nh.subscribe<my_object_position::ObjectDetection>(
            "/object_detection", 10, object_detection_cb);
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
    ros::ServiceClient play4_client =
        nh.serviceClient<std_srvs::Trigger>("play4");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // A valid local odometry origin is required before constructing any
    // relative mission coordinate or publishing the first setpoint.
    while (ros::ok() && !flag_init_position)
    {
        ROS_WARN_THROTTLE(2.5, "Waiting for valid local odometry...");
        ros::spinOnce();
        rate.sleep();
    }

    config.avoidance.obstacle_x += init_position_x_take_off;
    config.avoidance.obstacle_y += init_position_y_take_off;
    config.satellite.center_x = init_position_x_take_off + config.satellite_x;
    config.satellite.center_y = init_position_y_take_off + config.satellite_y;
    config.satellite.altitude = init_position_z_take_off + ALTITUDE;
    config.yellow.staging_setpoint = DesiredSetpoint(
        init_position_x_take_off + config.yellow_balloon_x,
        init_position_y_take_off + config.yellow_balloon_y,
        init_position_z_take_off + config.yellow_balloon_altitude,
        yaw);

    AvoidanceTask avoidance_task;
    SatelliteTask satellite_task;
    YellowBalloonTask yellow_task;
    if (!avoidance_task.configure(config.avoidance, error) ||
        !satellite_task.configure(config.satellite, error) ||
        !yellow_task.configure(config.yellow, error))
    {
        ROS_FATAL("Task module configuration failed: %s", error.c_str());
        return 1;
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =init_position_x_take_off + 0;
    pose.pose.position.y =init_position_y_take_off + 0;
    pose.pose.position.z =init_position_z_take_off + ALTITUDE;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = ros::Time::now();
     
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

    enum MissionPhase
    {
        START_SATELLITE,
        OUTBOUND_AVOIDANCE,
        YELLOW_BALLOON,
        RETURN_AVOIDANCE,
        HOLD,
        LANDING,
        FAILED_HOLD
    };

    MissionPhase phase = config.mission_mode == 3
        ? START_SATELLITE : OUTBOUND_AVOIDANCE;
    DesiredSetpoint command(
        init_position_x_take_off, init_position_y_take_off,
        init_position_z_take_off + ALTITUDE, yaw);
    const DesiredSetpoint test_goal(
        init_position_x_take_off + config.test_goal_x,
        init_position_y_take_off + config.test_goal_y,
        init_position_z_take_off + ALTITUDE, yaw);
    const DesiredSetpoint yellow_goal = config.yellow.staging_setpoint;
    const DesiredSetpoint home_goal(
        init_position_x_take_off, init_position_y_take_off,
        init_position_z_take_off + ALTITUDE, yaw);

    VehicleState vehicle = currentVehicleState();
    if (phase == START_SATELLITE)
    {
        if (!satellite_task.start(vehicle, error))
        {
            ROS_ERROR("Satellite task start failed: %s", error.c_str());
            phase = FAILED_HOLD;
        }
    }
    else
    {
        const DesiredSetpoint& target = config.mission_mode == 1
            ? test_goal : yellow_goal;
        if (!avoidance_task.start(vehicle, target,
                                  config.avoidance.orbit_direction, error))
        {
            ROS_ERROR("Outbound avoidance start failed: %s", error.c_str());
            phase = FAILED_HOLD;
        }
    }

    bool land_mode_sent = false;
    ros::Time last_land_request(0);
    while (ros::ok())
    {
        ros::spinOnce();
        const ros::Time now = ros::Time::now();
        vehicle = currentVehicleState();
        TaskUpdate update;

        if (phase == START_SATELLITE)
        {
            update = satellite_task.update(vehicle, satellite_detection, now);
            if (update.status != TaskStatus::FAILED)
            {
                command = update.setpoint;
            }
            if (update.request_play4)
            {
                std_srvs::Trigger play4_srv;
                const bool success = play4_client.call(play4_srv) &&
                                     play4_srv.response.success;
                satellite_task.handlePlay4Result(success, now);
            }
            if (update.status == TaskStatus::SUCCEEDED)
            {
                if (avoidance_task.start(
                        vehicle, yellow_goal,
                        config.avoidance.orbit_direction, error))
                {
                    phase = OUTBOUND_AVOIDANCE;
                    ROS_INFO("Satellite module complete. Starting avoidance to yellow area.");
                }
                else
                {
                    ROS_ERROR("Avoidance after satellite failed to start: %s",
                              error.c_str());
                    phase = FAILED_HOLD;
                }
            }
            else if (update.status == TaskStatus::FAILED)
            {
                ROS_ERROR("Satellite task failed: %s", update.error_reason.c_str());
                phase = FAILED_HOLD;
            }
        }
        else if (phase == OUTBOUND_AVOIDANCE)
        {
            update = avoidance_task.update(vehicle, now);
            if (update.status != TaskStatus::FAILED)
            {
                command = update.setpoint;
            }
            if (update.status == TaskStatus::SUCCEEDED)
            {
                if (config.mission_mode == 1)
                {
                    phase = HOLD;
                    ROS_INFO("Mode 1 obstacle test complete. Holding at test target.");
                }
                else if (yellow_task.start(vehicle, error))
                {
                    phase = YELLOW_BALLOON;
                    ROS_INFO("Yellow area reached. Starting yellow balloon module.");
                }
                else
                {
                    ROS_ERROR("Yellow balloon task failed to start: %s",
                              error.c_str());
                    phase = FAILED_HOLD;
                }
            }
            else if (update.status == TaskStatus::FAILED)
            {
                ROS_ERROR("Avoidance task failed: %s", update.error_reason.c_str());
                phase = FAILED_HOLD;
            }
        }
        else if (phase == YELLOW_BALLOON)
        {
            update = yellow_task.update(vehicle, yellow_detection, now);
            if (update.status != TaskStatus::FAILED)
            {
                command = update.setpoint;
            }
            if (update.status == TaskStatus::SUCCEEDED)
            {
                avoidance_task.reset();
                if (avoidance_task.configure(config.avoidance, error) &&
                    avoidance_task.start(
                        vehicle, home_goal,
                        -config.avoidance.orbit_direction, error))
                {
                    phase = RETURN_AVOIDANCE;
                    ROS_INFO("Yellow balloon module complete. Returning home with avoidance.");
                }
                else
                {
                    ROS_ERROR("Return avoidance failed to start: %s", error.c_str());
                    phase = FAILED_HOLD;
                }
            }
            else if (update.status == TaskStatus::FAILED)
            {
                ROS_ERROR("Yellow balloon task failed: %s",
                          update.error_reason.c_str());
                phase = FAILED_HOLD;
            }
        }
        else if (phase == RETURN_AVOIDANCE)
        {
            update = avoidance_task.update(vehicle, now);
            if (update.status != TaskStatus::FAILED)
            {
                command = update.setpoint;
            }
            if (update.status == TaskStatus::SUCCEEDED)
            {
                phase = config.auto_land_after_mission ? LANDING : HOLD;
                ROS_INFO("Home reached. %s",
                         phase == LANDING ? "Requesting AUTO.LAND." :
                                            "Holding at home.");
            }
            else if (update.status == TaskStatus::FAILED)
            {
                ROS_ERROR("Return avoidance failed: %s",
                          update.error_reason.c_str());
                phase = FAILED_HOLD;
            }
        }
        else if (phase == LANDING)
        {
            command = home_goal;
            if (!land_mode_sent &&
                now - last_land_request > ros::Duration(1.0))
            {
                mavros_msgs::SetMode land_mode;
                land_mode.request.custom_mode = "AUTO.LAND";
                land_mode_sent = set_mode_client.call(land_mode) &&
                                 land_mode.response.mode_sent;
                last_land_request = now;
                if (land_mode_sent)
                {
                    ROS_INFO("AUTO.LAND enabled.");
                }
                else
                {
                    ROS_WARN("Failed to enable AUTO.LAND. Holding home setpoint.");
                }
            }
        }
        else if (phase == HOLD)
        {
            ROS_INFO_THROTTLE(2.0, "Mission complete. Holding final setpoint.");
        }
        else if (phase == FAILED_HOLD)
        {
            ROS_ERROR_THROTTLE(1.0,
                "Mission module failed. Holding the last safe setpoint.");
        }

        if (phase != LANDING &&
            (current_state.mode != "OFFBOARD" || !current_state.armed))
        {
            ROS_WARN_THROTTLE(1.0,
                "OFFBOARD or armed state was lost during the mission. "
                "The test base keeps publishing but will not retake control automatically.");
        }

        applySetpoint(command, pose, now);
        local_pos_pub.publish(pose);
        ROS_INFO_THROTTLE(1.0,
            "mode=%d phase=%d satellite=%s avoidance=%s yellow=%s "
            "current=(%.2f, %.2f, %.2f) setpoint=(%.2f, %.2f, %.2f)",
            config.mission_mode, static_cast<int>(phase),
            satellite_task.stageName(), avoidance_task.stageName(),
            yellow_task.stageName(), vehicle.x, vehicle.y, vehicle.z,
            command.x, command.y, command.z);
        rate.sleep();
    }

    return 0;
}
