#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include <offboard_circle_modified/avoidance_task.h>

namespace offboard_test
{
namespace
{

const double kPi = 3.14159265358979323846;

double yawToward(double current_x, double current_y,
                 double target_x, double target_y,
                 double fallback_yaw)
{
    const double dx = target_x - current_x;
    const double dy = target_y - current_y;
    if (dx * dx + dy * dy < 1e-8)
    {
        return fallback_yaw;
    }
    return std::atan2(dy, dx);
}

}  // namespace

AvoidanceConfig::AvoidanceConfig()
    : obstacle_x(0.0), obstacle_y(0.0), safety_radius(1.0),
      orbit_angle_deg(180.0), orbit_angular_speed(0.1),
      orbit_direction(1), arrive_tolerance(0.12),
      orbit_tracking_tolerance(0.25)
{
}

AvoidanceTask::AvoidanceTask()
{
    reset();
}

bool AvoidanceTask::configure(const AvoidanceConfig& config,
                              std::string& error)
{
    if (config.safety_radius <= 0.05)
    {
        error = "obstacle safety radius must be greater than 0.05 m";
        return false;
    }
    if (config.orbit_angle_deg < 0.0 || config.orbit_angular_speed <= 0.0)
    {
        error = "avoidance arc angle/speed is invalid";
        return false;
    }
    if (config.arrive_tolerance <= 0.0 ||
        config.orbit_tracking_tolerance <= 0.0)
    {
        error = "avoidance tolerance is invalid";
        return false;
    }

    config_ = config;
    config_.orbit_direction = config_.orbit_direction >= 0 ? 1 : -1;
    error.clear();
    return true;
}

bool AvoidanceTask::directPathNeedsAvoidance(
    double start_x, double start_y, double goal_x, double goal_y,
    double& minimum_distance) const
{
    const double vx = goal_x - start_x;
    const double vy = goal_y - start_y;
    const double wx = config_.obstacle_x - start_x;
    const double wy = config_.obstacle_y - start_y;
    const double segment_length_sq = vx * vx + vy * vy;

    if (segment_length_sq < 1e-12)
    {
        minimum_distance = std::hypot(wx, wy);
        return minimum_distance <= config_.safety_radius;
    }

    double projection = (wx * vx + wy * vy) / segment_length_sq;
    projection = std::max(0.0, std::min(1.0, projection));
    const double closest_x = start_x + projection * vx;
    const double closest_y = start_y + projection * vy;
    minimum_distance = distance2D(
        closest_x, closest_y, config_.obstacle_x, config_.obstacle_y);
    return minimum_distance <= config_.safety_radius;
}

bool AvoidanceTask::start(const VehicleState& vehicle,
                          const DesiredSetpoint& target,
                          int direction,
                          std::string& error)
{
    reset();
    if (!vehicle.valid)
    {
        error = "cannot start avoidance without valid odometry";
        stage_ = FAILED;
        return false;
    }

    target_ = target;
    direction_ = direction >= 0 ? 1 : -1;
    const double start_radius = distance2D(
        vehicle.x, vehicle.y, config_.obstacle_x, config_.obstacle_y);
    const double target_radius = distance2D(
        target_.x, target_.y, config_.obstacle_x, config_.obstacle_y);
    if (start_radius <= config_.safety_radius ||
        target_radius <= config_.safety_radius)
    {
        error = "avoidance start or target lies inside the safety circle";
        stage_ = FAILED;
        return false;
    }

    if (!directPathNeedsAvoidance(vehicle.x, vehicle.y,
                                  target_.x, target_.y,
                                  path_min_distance_))
    {
        stage_ = TO_TARGET;
        error.clear();
        ROS_INFO("Direct path is clear: minimum obstacle distance %.3f m "
                 "> safety radius %.3f m.",
                 path_min_distance_, config_.safety_radius);
        return true;
    }

    const double dx = config_.obstacle_x - vehicle.x;
    const double dy = config_.obstacle_y - vehicle.y;
    const double distance_to_center = std::hypot(dx, dy);
    if (distance_to_center < 1e-6)
    {
        error = "cannot calculate obstacle approach direction";
        stage_ = FAILED;
        return false;
    }

    approach_x_ = config_.obstacle_x -
        dx / distance_to_center * config_.safety_radius;
    approach_y_ = config_.obstacle_y -
        dy / distance_to_center * config_.safety_radius;
    orbit_start_theta_ = std::atan2(
        approach_y_ - config_.obstacle_y,
        approach_x_ - config_.obstacle_x);

    double planned_angle = std::fabs(config_.orbit_angle_deg) * kPi / 180.0;
    const double one_degree = kPi / 180.0;
    const double maximum_angle = planned_angle + 2.0 * kPi;
    while (planned_angle <= maximum_angle)
    {
        const double candidate_theta = orbit_start_theta_ +
            static_cast<double>(direction_) * planned_angle;
        const double exit_x = config_.obstacle_x +
            config_.safety_radius * std::cos(candidate_theta);
        const double exit_y = config_.obstacle_y +
            config_.safety_radius * std::sin(candidate_theta);
        const double outward_dot =
            (target_.x - exit_x) * (exit_x - config_.obstacle_x) +
            (target_.y - exit_y) * (exit_y - config_.obstacle_y);
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
    last_update_time_ = ros::Time(0);
    error.clear();

    ROS_WARN("Avoidance required: path minimum distance %.3f m, safety "
             "radius %.3f m.", path_min_distance_, config_.safety_radius);
    ROS_INFO("Approach point=(%.3f, %.3f), planned arc=%.1f deg, "
             "direction=%s.", approach_x_, approach_y_,
             planned_angle * 180.0 / kPi,
             direction_ > 0 ? "CCW" : "CW");
    return true;
}

TaskUpdate AvoidanceTask::update(const VehicleState& vehicle,
                                 const ros::Time& now)
{
    TaskUpdate result;
    result.setpoint = target_;

    if (stage_ == IDLE)
    {
        result.status = TaskStatus::IDLE;
        return result;
    }
    if (stage_ == FAILED)
    {
        result.status = TaskStatus::FAILED;
        result.error_reason = "avoidance task is in FAILED state";
        return result;
    }
    if (!vehicle.valid)
    {
        stage_ = FAILED;
        result.status = TaskStatus::FAILED;
        result.error_reason = "odometry became invalid during avoidance";
        return result;
    }
    if (stage_ == COMPLETE)
    {
        result.status = TaskStatus::SUCCEEDED;
        return result;
    }

    double dt = 0.0;
    if (!last_update_time_.isZero())
    {
        dt = std::min(0.2, std::max(0.0, (now - last_update_time_).toSec()));
    }
    last_update_time_ = now;

    result.status = TaskStatus::RUNNING;
    if (stage_ == APPROACH)
    {
        result.setpoint = DesiredSetpoint(
            approach_x_, approach_y_, target_.z,
            yawToward(vehicle.x, vehicle.y,
                      config_.obstacle_x, config_.obstacle_y, vehicle.yaw));
        if (distance3D(vehicle.x, vehicle.y, vehicle.z,
                       approach_x_, approach_y_, target_.z) <=
            config_.arrive_tolerance)
        {
            stage_ = ORBIT;
            orbit_theta_ = orbit_start_theta_;
            ROS_INFO("Obstacle approach point reached. Starting arc.");
        }
        return result;
    }

    if (stage_ == ORBIT)
    {
        const double desired_x = config_.obstacle_x +
            config_.safety_radius * std::cos(orbit_theta_);
        const double desired_y = config_.obstacle_y +
            config_.safety_radius * std::sin(orbit_theta_);
        const double tracking_error = distance2D(
            vehicle.x, vehicle.y, desired_x, desired_y);

        if (tracking_error <= config_.orbit_tracking_tolerance)
        {
            const double step = config_.orbit_angular_speed * dt;
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
                tracking_error, config_.orbit_tracking_tolerance);
        }

        result.setpoint = DesiredSetpoint(
            config_.obstacle_x + config_.safety_radius * std::cos(orbit_theta_),
            config_.obstacle_y + config_.safety_radius * std::sin(orbit_theta_),
            target_.z,
            yawToward(vehicle.x, vehicle.y,
                      config_.obstacle_x, config_.obstacle_y, vehicle.yaw));
        const bool angle_finished = direction_ > 0
            ? orbit_theta_ >= orbit_target_theta_ - 1e-8
            : orbit_theta_ <= orbit_target_theta_ + 1e-8;
        if (angle_finished &&
            distance3D(vehicle.x, vehicle.y, vehicle.z,
                       result.setpoint.x, result.setpoint.y,
                       target_.z) <= config_.arrive_tolerance)
        {
            stage_ = TO_TARGET;
            ROS_INFO("Avoidance arc completed. Flying to leg target.");
        }
        return result;
    }

    result.setpoint.yaw = yawToward(
        vehicle.x, vehicle.y, target_.x, target_.y, vehicle.yaw);
    if (distance3D(vehicle.x, vehicle.y, vehicle.z,
                   target_.x, target_.y, target_.z) <=
        config_.arrive_tolerance)
    {
        stage_ = COMPLETE;
        result.status = TaskStatus::SUCCEEDED;
        ROS_INFO("Navigation leg target reached.");
    }
    return result;
}

void AvoidanceTask::reset()
{
    stage_ = IDLE;
    target_ = DesiredSetpoint();
    approach_x_ = 0.0;
    approach_y_ = 0.0;
    orbit_theta_ = 0.0;
    orbit_start_theta_ = 0.0;
    orbit_target_theta_ = 0.0;
    path_min_distance_ = 0.0;
    direction_ = 1;
    last_update_time_ = ros::Time(0);
}

const char* AvoidanceTask::stageName() const
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

}  // namespace offboard_test
