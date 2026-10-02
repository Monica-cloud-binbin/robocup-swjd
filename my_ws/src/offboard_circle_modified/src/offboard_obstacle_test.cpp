#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace
{

const double kPi = 3.14159265358979323846;

double ALTITUDE = 0.825;
mavros_msgs::State current_state;
tf::Quaternion quat;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
bool flag_init_position = false;
nav_msgs::Odometry local_pos;

struct Config
{
    double obstacle_x;
    double obstacle_y;
    double goal_x;
    double goal_y;
    double obstacle_safe_distance;
    double orbit_angle_deg;
    double orbit_angular_speed;
    int orbit_direction;
    double arrive_tolerance;
    double orbit_tracking_tolerance;
    double goal_hold_time;
    double land_hold_time;
    bool return_home_after_goal;
    bool auto_land;
};

Config config;

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

double distance2D(double x1, double y1, double x2, double y2)
{
    return std::hypot(x1 - x2, y1 - y2);
}

double distance3D(double x1, double y1, double z1,
                  double x2, double y2, double z2)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    const double dz = z1 - z2;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void setYawToward(geometry_msgs::PoseStamped& pose,
                  double current_x, double current_y,
                  double target_x, double target_y)
{
    const double dx = target_x - current_x;
    const double dy = target_y - current_y;
    if (dx * dx + dy * dy < 1e-8)
    {
        return;
    }

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(dy, dx));
    tf::quaternionTFToMsg(q, pose.pose.orientation);
}

bool directPathNeedsAvoidance(double start_x, double start_y,
                              double goal_x, double goal_y,
                              double obstacle_x, double obstacle_y,
                              double safety_radius,
                              double& minimum_distance)
{
    const double vx = goal_x - start_x;
    const double vy = goal_y - start_y;
    const double wx = obstacle_x - start_x;
    const double wy = obstacle_y - start_y;
    const double segment_length_sq = vx * vx + vy * vy;

    if (segment_length_sq < 1e-12)
    {
        minimum_distance = std::hypot(wx, wy);
        return minimum_distance <= safety_radius;
    }

    double projection = (wx * vx + wy * vy) / segment_length_sq;
    projection = std::max(0.0, std::min(1.0, projection));

    const double closest_x = start_x + projection * vx;
    const double closest_y = start_y + projection * vy;
    minimum_distance = distance2D(
        closest_x, closest_y, obstacle_x, obstacle_y);
    return minimum_distance <= safety_radius;
}

class AvoidanceLeg
{
public:
    enum Stage
    {
        IDLE,
        APPROACH,
        ORBIT,
        TO_TARGET,
        COMPLETE,
        FAILED
    };

    AvoidanceLeg()
        : stage_(IDLE), center_x_(0.0), center_y_(0.0), radius_(0.0),
          target_x_(0.0), target_y_(0.0), target_z_(0.0),
          approach_x_(0.0), approach_y_(0.0), orbit_theta_(0.0),
          orbit_start_theta_(0.0), orbit_target_theta_(0.0),
          direction_(1), angular_speed_(0.1), arrive_tolerance_(0.1),
          tracking_tolerance_(0.25), path_min_distance_(0.0)
    {
    }

    bool start(double start_x, double start_y,
               double target_x, double target_y, double target_z,
               double center_x, double center_y,
               double radius, double orbit_angle_deg,
               double angular_speed, int direction,
               double arrive_tolerance, double tracking_tolerance)
    {
        center_x_ = center_x;
        center_y_ = center_y;
        radius_ = radius;
        target_x_ = target_x;
        target_y_ = target_y;
        target_z_ = target_z;
        direction_ = direction >= 0 ? 1 : -1;
        angular_speed_ = angular_speed;
        arrive_tolerance_ = arrive_tolerance;
        tracking_tolerance_ = tracking_tolerance;

        const double start_radius = distance2D(
            start_x, start_y, center_x_, center_y_);
        const double target_radius = distance2D(
            target_x_, target_y_, center_x_, center_y_);
        if (start_radius <= radius_ || target_radius <= radius_)
        {
            ROS_ERROR("Avoidance leg invalid: start or target is inside the "
                      "obstacle safety circle (start=%.3f, target=%.3f, "
                      "radius=%.3f).",
                      start_radius, target_radius, radius_);
            stage_ = FAILED;
            return false;
        }

        if (!directPathNeedsAvoidance(
                start_x, start_y, target_x_, target_y_, center_x_, center_y_,
                radius_, path_min_distance_))
        {
            stage_ = TO_TARGET;
            ROS_INFO("Direct path is clear: minimum obstacle distance %.3f m "
                     "> safety radius %.3f m.",
                     path_min_distance_, radius_);
            return true;
        }

        const double dx = center_x_ - start_x;
        const double dy = center_y_ - start_y;
        const double distance_to_center = std::hypot(dx, dy);
        if (distance_to_center < 1e-6)
        {
            ROS_ERROR("Cannot calculate obstacle approach direction.");
            stage_ = FAILED;
            return false;
        }

        approach_x_ = center_x_ - dx / distance_to_center * radius_;
        approach_y_ = center_y_ - dy / distance_to_center * radius_;
        orbit_start_theta_ = std::atan2(
            approach_y_ - center_y_, approach_x_ - center_x_);

        double planned_angle = std::fabs(orbit_angle_deg) * kPi / 180.0;
        const double one_degree = kPi / 180.0;
        const double maximum_angle = planned_angle + 2.0 * kPi;

        // A fixed arc can end on the obstacle-facing side of the target. Extend
        // it only as much as needed so the final straight segment points away
        // from the safety circle.
        while (planned_angle <= maximum_angle)
        {
            const double candidate_theta = orbit_start_theta_ +
                static_cast<double>(direction_) * planned_angle;
            const double exit_x = center_x_ + radius_ * std::cos(candidate_theta);
            const double exit_y = center_y_ + radius_ * std::sin(candidate_theta);
            const double outward_dot =
                (target_x_ - exit_x) * (exit_x - center_x_) +
                (target_y_ - exit_y) * (exit_y - center_y_);
            if (outward_dot >= -1e-6)
            {
                break;
            }
            planned_angle += one_degree;
        }

        orbit_target_theta_ = orbit_start_theta_ +
            static_cast<double>(direction_) * planned_angle;
        orbit_theta_ = orbit_start_theta_;
        stage_ = APPROACH;

        ROS_WARN("Avoidance required: path minimum distance %.3f m, safety "
                 "radius %.3f m.", path_min_distance_, radius_);
        ROS_INFO("Approach point=(%.3f, %.3f), planned arc=%.1f deg, "
                 "direction=%s.",
                 approach_x_, approach_y_, planned_angle * 180.0 / kPi,
                 direction_ > 0 ? "CCW" : "CW");
        return true;
    }

    void update(double current_x, double current_y, double current_z,
                double dt, geometry_msgs::PoseStamped& command)
    {
        if (stage_ == APPROACH)
        {
            command.pose.position.x = approach_x_;
            command.pose.position.y = approach_y_;
            command.pose.position.z = target_z_;
            setYawToward(command, current_x, current_y, center_x_, center_y_);

            if (distance3D(current_x, current_y, current_z,
                           approach_x_, approach_y_, target_z_) <=
                arrive_tolerance_)
            {
                stage_ = ORBIT;
                orbit_theta_ = orbit_start_theta_;
                ROS_INFO("Obstacle approach point reached. Starting arc.");
            }
            return;
        }

        if (stage_ == ORBIT)
        {
            const double desired_x = center_x_ + radius_ * std::cos(orbit_theta_);
            const double desired_y = center_y_ + radius_ * std::sin(orbit_theta_);
            const double tracking_error = distance2D(
                current_x, current_y, desired_x, desired_y);

            if (tracking_error <= tracking_tolerance_)
            {
                const double step = angular_speed_ * std::max(0.0, dt);
                if (direction_ > 0)
                {
                    orbit_theta_ = std::min(orbit_theta_ + step,
                                            orbit_target_theta_);
                }
                else
                {
                    orbit_theta_ = std::max(orbit_theta_ - step,
                                            orbit_target_theta_);
                }
            }
            else
            {
                ROS_WARN_THROTTLE(1.0,
                    "Arc setpoint paused: tracking error %.3f m > %.3f m.",
                    tracking_error, tracking_tolerance_);
            }

            command.pose.position.x = center_x_ + radius_ * std::cos(orbit_theta_);
            command.pose.position.y = center_y_ + radius_ * std::sin(orbit_theta_);
            command.pose.position.z = target_z_;
            setYawToward(command, current_x, current_y, center_x_, center_y_);

            const bool angle_finished = direction_ > 0
                ? orbit_theta_ >= orbit_target_theta_ - 1e-8
                : orbit_theta_ <= orbit_target_theta_ + 1e-8;
            if (angle_finished &&
                distance3D(current_x, current_y, current_z,
                           command.pose.position.x, command.pose.position.y,
                           target_z_) <= arrive_tolerance_)
            {
                stage_ = TO_TARGET;
                ROS_INFO("Avoidance arc completed. Flying to leg target.");
            }
            return;
        }

        if (stage_ == TO_TARGET)
        {
            command.pose.position.x = target_x_;
            command.pose.position.y = target_y_;
            command.pose.position.z = target_z_;
            setYawToward(command, current_x, current_y, target_x_, target_y_);

            if (distance3D(current_x, current_y, current_z,
                           target_x_, target_y_, target_z_) <=
                arrive_tolerance_)
            {
                stage_ = COMPLETE;
                ROS_INFO("Navigation leg target reached.");
            }
        }
    }

    bool complete() const { return stage_ == COMPLETE; }
    bool failed() const { return stage_ == FAILED; }

    const char* stageName() const
    {
        switch (stage_)
        {
            case IDLE: return "IDLE";
            case APPROACH: return "APPROACH";
            case ORBIT: return "ORBIT";
            case TO_TARGET: return "TO_TARGET";
            case COMPLETE: return "COMPLETE";
            case FAILED: return "FAILED";
        }
        return "UNKNOWN";
    }

private:
    Stage stage_;
    double center_x_;
    double center_y_;
    double radius_;
    double target_x_;
    double target_y_;
    double target_z_;
    double approach_x_;
    double approach_y_;
    double orbit_theta_;
    double orbit_start_theta_;
    double orbit_target_theta_;
    int direction_;
    double angular_speed_;
    double arrive_tolerance_;
    double tracking_tolerance_;
    double path_min_distance_;
};

void loadParameters(ros::NodeHandle& private_nh)
{
    private_nh.param<double>("ALTITUDE", ALTITUDE, 0.825);
    private_nh.param<double>("obstacle_x", config.obstacle_x, 1.8);
    private_nh.param<double>("obstacle_y", config.obstacle_y, 3.9);
    private_nh.param<double>("goal_x", config.goal_x, 4.3);
    private_nh.param<double>("goal_y", config.goal_y, 3.9);
    private_nh.param<double>("obstacle_safe_distance",
                             config.obstacle_safe_distance, 1.0);
    private_nh.param<double>("orbit_angle_deg", config.orbit_angle_deg, 180.0);
    private_nh.param<double>("orbit_angular_speed",
                             config.orbit_angular_speed, 0.10);
    private_nh.param<int>("orbit_direction", config.orbit_direction, 1);
    private_nh.param<double>("arrive_tolerance",
                             config.arrive_tolerance, 0.12);
    private_nh.param<double>("orbit_tracking_tolerance",
                             config.orbit_tracking_tolerance, 0.25);
    private_nh.param<double>("goal_hold_time", config.goal_hold_time, 3.0);
    private_nh.param<double>("land_hold_time", config.land_hold_time, 3.0);
    private_nh.param<bool>("return_home_after_goal",
                           config.return_home_after_goal, false);
    private_nh.param<bool>("auto_land", config.auto_land, false);

    config.orbit_direction = config.orbit_direction >= 0 ? 1 : -1;
}

bool validateParameters(const Config& config)
{
    bool valid = true;
    if (ALTITUDE <= 0.0)
    {
        ROS_ERROR("ALTITUDE must be positive.");
        valid = false;
    }
    if (config.obstacle_safe_distance <= 0.05)
    {
        ROS_ERROR("obstacle_safe_distance must be greater than 0.05 m.");
        valid = false;
    }
    if (config.orbit_angle_deg < 0.0 || config.orbit_angular_speed <= 0.0)
    {
        ROS_ERROR("Orbit angle must be non-negative and angular speed positive.");
        valid = false;
    }
    if (config.arrive_tolerance <= 0.0 ||
        config.orbit_tracking_tolerance <= 0.0)
    {
        ROS_ERROR("Tolerance parameters are invalid.");
        valid = false;
    }
    if (config.auto_land && !config.return_home_after_goal)
    {
        ROS_WARN("auto_land is ignored unless return_home_after_goal is true.");
    }
    return valid;
}

}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_multi_position");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");  
    // 加载参数
    loadParameters(private_nh);

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

    if (!validateParameters(config))
    {
        ROS_FATAL("Obstacle test parameters are invalid.");
        return 1;
    }

    const double task_z = init_position_z_take_off + ALTITUDE;
    const double obstacle_global_x = init_position_x_take_off + config.obstacle_x;
    const double obstacle_global_y = init_position_y_take_off + config.obstacle_y;
    const double goal_global_x = init_position_x_take_off + config.goal_x;
    const double goal_global_y = init_position_y_take_off + config.goal_y;

    if (distance2D(init_position_x_take_off, init_position_y_take_off,
                   obstacle_global_x, obstacle_global_y) <=
            config.obstacle_safe_distance ||
        distance2D(goal_global_x, goal_global_y,
                   obstacle_global_x, obstacle_global_y) <=
            config.obstacle_safe_distance)
    {
        ROS_FATAL("Origin or goal lies inside the configured safety circle.");
        return 1;
    }

    ROS_INFO("Obstacle=(%.3f, %.3f), goal=(%.3f, %.3f), task z=%.3f",
             obstacle_global_x, obstacle_global_y,
             goal_global_x, goal_global_y, task_z);

    geometry_msgs::PoseStamped command = pose;
    ROS_INFO("Takeoff finished. Starting obstacle test.");

    enum MissionPhase
    {
        OUTBOUND,
        GOAL_HOLD,
        RETURN_HOME,
        HOME_HOLD,
        LANDING,
        FINISHED
    };

    MissionPhase phase = OUTBOUND;
    AvoidanceLeg active_leg;
    if (!active_leg.start(
            local_pos.pose.pose.position.x,
            local_pos.pose.pose.position.y,
            goal_global_x, goal_global_y, task_z,
            obstacle_global_x, obstacle_global_y,
            config.obstacle_safe_distance, config.orbit_angle_deg,
            config.orbit_angular_speed, config.orbit_direction,
            config.arrive_tolerance, config.orbit_tracking_tolerance))
    {
        ROS_FATAL("Failed to create outbound avoidance leg.");
        return 1;
    }

    ros::Time phase_start;
    ros::Time last_loop_time = ros::Time::now();
    bool land_command_sent = false;

    while (ros::ok())
    {
        ros::spinOnce();
        const ros::Time now = ros::Time::now();
        const double dt = std::min(0.2, (now - last_loop_time).toSec());
        last_loop_time = now;

        if (phase != LANDING &&
            (current_state.mode != "OFFBOARD" || !current_state.armed))
        {
            ROS_WARN_THROTTLE(1.0,
                "OFFBOARD or armed state was lost. The node will not "
                "automatically take control again during the mission.");
        }

        const double current_x = local_pos.pose.pose.position.x;
        const double current_y = local_pos.pose.pose.position.y;
        const double current_z = local_pos.pose.pose.position.z;

        if (phase == OUTBOUND || phase == RETURN_HOME)
        {
            active_leg.update(current_x, current_y, current_z, dt, command);
            if (active_leg.failed())
            {
                ROS_ERROR_THROTTLE(1.0,
                    "Navigation leg failed. Holding current setpoint.");
                phase = FINISHED;
            }
            else if (active_leg.complete())
            {
                phase_start = now;
                phase = phase == OUTBOUND ? GOAL_HOLD : HOME_HOLD;
            }
        }
        else if (phase == GOAL_HOLD)
        {
            command.pose.position.x = goal_global_x;
            command.pose.position.y = goal_global_y;
            command.pose.position.z = task_z;
            if (now - phase_start >= ros::Duration(config.goal_hold_time))
            {
                if (config.return_home_after_goal)
                {
                    if (active_leg.start(
                            current_x, current_y,
                            init_position_x_take_off,
                            init_position_y_take_off, task_z,
                            obstacle_global_x, obstacle_global_y,
                            config.obstacle_safe_distance,
                            config.orbit_angle_deg,
                            config.orbit_angular_speed,
                            -config.orbit_direction,
                            config.arrive_tolerance,
                            config.orbit_tracking_tolerance))
                    {
                        phase = RETURN_HOME;
                        ROS_INFO("Goal hold complete. Returning home.");
                    }
                    else
                    {
                        phase = FINISHED;
                        ROS_ERROR("Return path planning failed. Holding at goal.");
                    }
                }
                else
                {
                    phase = FINISHED;
                    ROS_INFO("Obstacle test complete. Holding at goal.");
                }
            }
        }
        else if (phase == HOME_HOLD)
        {
            command.pose.position.x = init_position_x_take_off;
            command.pose.position.y = init_position_y_take_off;
            command.pose.position.z = task_z;
            if (now - phase_start >= ros::Duration(config.land_hold_time))
            {
                if (config.auto_land)
                {
                    phase = LANDING;
                }
                else
                {
                    phase = FINISHED;
                    ROS_INFO("Returned home. Holding because auto_land=false.");
                }
            }
        }
        else if (phase == LANDING)
        {
            command.pose.position.x = init_position_x_take_off;
            command.pose.position.y = init_position_y_take_off;
            command.pose.position.z = task_z;
            if (!land_command_sent)
            {
                mavros_msgs::SetMode land_mode;
                land_mode.request.custom_mode = "AUTO.LAND";
                if (set_mode_client.call(land_mode) &&
                    land_mode.response.mode_sent)
                {
                    land_command_sent = true;
                    ROS_INFO("AUTO.LAND enabled.");
                }
                else
                {
                    ROS_WARN_THROTTLE(1.0, "Failed to enable AUTO.LAND.");
                }
            }
        }
        else if (phase == FINISHED)
        {
            ROS_INFO_THROTTLE(2.0, "Test finished; holding final setpoint.");
        }

        ROS_INFO_THROTTLE(1.0,
            "phase=%d leg=%s current=(%.2f, %.2f, %.2f) "
            "setpoint=(%.2f, %.2f, %.2f)",
            static_cast<int>(phase), active_leg.stageName(),
            current_x, current_y, current_z,
            command.pose.position.x, command.pose.position.y,
            command.pose.position.z);

        command.header.stamp = now;
        local_pos_pub.publish(command);
        rate.sleep();
    }

    return 0;
}
