#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include <offboard_circle_modified/yellow_balloon_task.h>

namespace offboard_test
{
namespace
{

const double kPi = 3.14159265358979323846;

bool finite(double value)
{
    return std::isfinite(value);
}

double normalizeAngle(double angle)
{
    while (angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

double clampValue(double value, double lower, double upper)
{
    return std::max(lower, std::min(value, upper));
}

}  // namespace

YellowBalloonConfig::YellowBalloonConfig()
    : buffer_distance(1.5), search_yaw_rate(0.2),
      image_center_x(320.0), image_center_y(240.0),
      alignment_tolerance_x(30.0), alignment_tolerance_y(30.0),
      alignment_count_required(5), yaw_kp(0.001),
      yaw_control_sign(-1.0), max_yaw_rate(0.25),
      vertical_kp(0.001), vertical_control_sign(-1.0),
      max_vertical_speed(0.20), min_altitude(0.5), max_altitude(2.5),
      target_x_offset(4.3), target_y_offset(3.9),
      target_altitude_offset(1.2),
      detection_timeout(0.5), approach_speed(0.25),
      area_ratio_threshold(0.08), max_approach_distance(3.0),
      approach_timeout(20.0), contact_speed(0.25),
      contact_distance(0.5), contact_arrive_tolerance(0.12),
      contact_timeout(8.0)
{
}

YellowBalloonTask::YellowBalloonTask()
    : configured_(false)
{
    reset();
}

bool YellowBalloonTask::configure(const YellowBalloonConfig& config,
                                  std::string& error)
{
    configured_ = false;
    const DesiredSetpoint& target = config.staging_setpoint;
    if (!finite(target.x) || !finite(target.y) || !finite(target.z) ||
        !finite(target.yaw))
    {
        error = "yellow balloon target contains a non-finite value";
        return false;
    }
    if (!finite(config.buffer_distance) || config.buffer_distance <= 0.0 ||
        !finite(config.search_yaw_rate) || config.search_yaw_rate == 0.0 ||
        !finite(config.image_center_x) || !finite(config.image_center_y) ||
        config.image_center_x < 0.0 || config.image_center_y < 0.0 ||
        !finite(config.alignment_tolerance_x) ||
        !finite(config.alignment_tolerance_y) ||
        config.alignment_tolerance_x < 0.0 ||
        config.alignment_tolerance_y < 0.0 ||
        config.alignment_count_required <= 0)
    {
        error = "yellow balloon buffer/search/image alignment parameters are invalid";
        return false;
    }
    if (!finite(config.yaw_kp) || config.yaw_kp < 0.0 ||
        !finite(config.yaw_control_sign) || config.yaw_control_sign == 0.0 ||
        !finite(config.max_yaw_rate) || config.max_yaw_rate <= 0.0 ||
        !finite(config.vertical_kp) || config.vertical_kp < 0.0 ||
        !finite(config.vertical_control_sign) ||
        config.vertical_control_sign == 0.0 ||
        !finite(config.max_vertical_speed) ||
        config.max_vertical_speed <= 0.0)
    {
        error = "yellow balloon alignment gains/signs/limits are invalid";
        return false;
    }
    if (!finite(config.min_altitude) || !finite(config.max_altitude) ||
        !finite(config.target_x_offset) || !finite(config.target_y_offset) ||
        !finite(config.target_altitude_offset) ||
        config.min_altitude < 0.0 || config.min_altitude > config.max_altitude ||
        config.target_altitude_offset < config.min_altitude ||
        config.target_altitude_offset > config.max_altitude)
    {
        error = "yellow balloon altitude limits are invalid or exclude the target altitude";
        return false;
    }
    const AvoidanceConfig& avoidance = config.approach_avoidance;
    if (!finite(avoidance.obstacle_x) || !finite(avoidance.obstacle_y) ||
        !finite(avoidance.safety_radius) ||
        !finite(avoidance.orbit_angle_deg) ||
        !finite(avoidance.orbit_angular_speed) ||
        !finite(avoidance.arrive_tolerance) ||
        !finite(avoidance.orbit_tracking_tolerance))
    {
        error = "yellow balloon internal avoidance contains a non-finite value";
        return false;
    }
    if (!finite(config.detection_timeout) || config.detection_timeout <= 0.0 ||
        !finite(config.approach_speed) || config.approach_speed <= 0.0 ||
        !finite(config.area_ratio_threshold) ||
        config.area_ratio_threshold <= 0.0 ||
        config.area_ratio_threshold > 1.0 ||
        !finite(config.max_approach_distance) ||
        config.max_approach_distance <= 0.0 ||
        !finite(config.approach_timeout) || config.approach_timeout <= 0.0)
    {
        error = "yellow balloon detection/approach parameters are invalid";
        return false;
    }
    if (!finite(config.contact_speed) || config.contact_speed <= 0.0 ||
        !finite(config.contact_distance) || config.contact_distance <= 0.0 ||
        !finite(config.contact_arrive_tolerance) ||
        config.contact_arrive_tolerance <= 0.0 ||
        config.contact_arrive_tolerance >= config.contact_distance ||
        !finite(config.contact_timeout) || config.contact_timeout <= 0.0)
    {
        error = "yellow balloon contact parameters are invalid";
        return false;
    }

    config_ = config;
    const double x_origin = target.x - config_.target_x_offset;
    const double y_origin = target.y - config_.target_y_offset;
    const double altitude_origin = target.z - config_.target_altitude_offset;
    if (!finite(x_origin) || !finite(y_origin) || !finite(altitude_origin))
    {
        error = "yellow balloon takeoff-origin conversion is not finite";
        return false;
    }
    config_.approach_avoidance.obstacle_x += x_origin;
    config_.approach_avoidance.obstacle_y += y_origin;
    config_.min_altitude += altitude_origin;
    config_.max_altitude += altitude_origin;

    std::string avoidance_error;
    if (!approach_avoidance_.configure(config_.approach_avoidance,
                                       avoidance_error))
    {
        error = "yellow balloon approach avoidance config: " + avoidance_error;
        return false;
    }

    configured_ = true;
    error.clear();
    return true;
}

bool YellowBalloonTask::start(const VehicleState& vehicle,
                              std::string& error)
{
    reset();
    if (!configured_)
    {
        error = "yellow balloon task is not configured";
        fail(error);
        return false;
    }
    if (!vehicle.valid)
    {
        error = "cannot start yellow balloon task without valid odometry";
        fail(error);
        return false;
    }

    const double target_x = config_.staging_setpoint.x;
    const double target_y = config_.staging_setpoint.y;
    const double dx = target_x - vehicle.x;
    const double dy = target_y - vehicle.y;
    const double distance = std::hypot(dx, dy);
    if (!finite(vehicle.x) || !finite(vehicle.y) || !finite(vehicle.z) ||
        !finite(vehicle.yaw) || !finite(distance) || distance <= 1e-6)
    {
        error = "yellow balloon start and target positions are invalid or coincident";
        fail(error);
        return false;
    }
    if (distance <= config_.buffer_distance)
    {
        error = "yellow balloon task start lies inside or on the buffer circle";
        fail(error);
        return false;
    }

    buffer_setpoint_ = DesiredSetpoint(
        target_x - config_.buffer_distance * dx / distance,
        target_y - config_.buffer_distance * dy / distance,
        config_.staging_setpoint.z,
        std::atan2(dy, dx));
    command_ = DesiredSetpoint(vehicle.x, vehicle.y, vehicle.z, vehicle.yaw);

    if (!approach_avoidance_.start(
            vehicle, buffer_setpoint_,
            config_.approach_avoidance.orbit_direction, error))
    {
        error = "yellow balloon buffer avoidance start failed: " + error;
        fail(error);
        return false;
    }

    stage_ = MOVE_TO_BUFFER;
    start_hold_pending_ = true;
    last_update_time_ = ros::Time(0);
    error.clear();
    ROS_INFO("Yellow balloon task: start=(%.3f, %.3f), target=(%.3f, %.3f), "
             "buffer=(%.3f, %.3f), radius=%.3f m.",
             vehicle.x, vehicle.y, target_x, target_y,
             buffer_setpoint_.x, buffer_setpoint_.y, config_.buffer_distance);
    ROS_INFO("Yellow balloon internal avoidance started toward the buffer point.");
    return true;
}

bool YellowBalloonTask::detectionIsFresh(
    const DetectionSnapshot& detection, const ros::Time& now) const
{
    if (!detection.valid || !finite(detection.area_ratio) ||
        detection.message_time.isZero() ||
        now < detection.message_time)
    {
        return false;
    }
    return now - detection.message_time <=
        ros::Duration(config_.detection_timeout);
}

bool YellowBalloonTask::detectionIsCentered(
    const DetectionSnapshot& detection) const
{
    return std::fabs(detection.center_x - config_.image_center_x) <=
               config_.alignment_tolerance_x &&
           std::fabs(detection.center_y - config_.image_center_y) <=
               config_.alignment_tolerance_y;
}

void YellowBalloonTask::fail(const std::string& reason)
{
    stage_ = FAILED;
    failure_reason_ = reason;
    ROS_ERROR("Yellow balloon task failed: %s", reason.c_str());
}

double YellowBalloonTask::updateDt(const ros::Time& now)
{
    double dt = 0.0;
    if (!last_update_time_.isZero() && now >= last_update_time_)
    {
        dt = clampValue((now - last_update_time_).toSec(), 0.0, 0.2);
    }
    last_update_time_ = now;
    return dt;
}

void YellowBalloonTask::beginSearch(
    const DesiredSetpoint& hold_setpoint,
    const DetectionSnapshot& detection,
    const ros::Time& now)
{
    search_hold_setpoint_ = hold_setpoint;
    command_ = hold_setpoint;
    aligned_detection_count_ = 0;
    last_processed_sequence_ = detection.sequence;
    stage_start_time_ = now;
    stage_ = SEARCH;
    ROS_INFO("Yellow balloon task entered SEARCH at (%.3f, %.3f, %.3f).",
             command_.x, command_.y, command_.z);
}

TaskUpdate YellowBalloonTask::update(
    const VehicleState& vehicle, const DetectionSnapshot& detection,
    const ros::Time& now)
{
    TaskUpdate result;
    result.setpoint = command_;

    if (stage_ == IDLE)
    {
        result.status = TaskStatus::IDLE;
        return result;
    }
    if (stage_ == FAILED)
    {
        result.status = TaskStatus::FAILED;
        result.error_reason = failure_reason_.empty()
            ? "yellow balloon task is in FAILED state" : failure_reason_;
        return result;
    }
    if (!vehicle.valid || !finite(vehicle.x) || !finite(vehicle.y) ||
        !finite(vehicle.z) || !finite(vehicle.yaw))
    {
        fail("odometry became invalid during yellow balloon task");
        result.status = TaskStatus::FAILED;
        result.error_reason = failure_reason_;
        return result;
    }
    if (stage_ == FINISHED)
    {
        result.status = TaskStatus::SUCCEEDED;
        return result;
    }

    const double dt = updateDt(now);
    result.status = TaskStatus::RUNNING;
    if (stage_ == MOVE_TO_BUFFER)
    {
        if (start_hold_pending_)
        {
            start_hold_pending_ = false;
            result.setpoint = command_;
            return result;
        }
        const TaskUpdate avoidance_update =
            approach_avoidance_.update(vehicle, now);
        if (avoidance_update.status == TaskStatus::FAILED)
        {
            fail("internal avoidance failed: " +
                 avoidance_update.error_reason);
            result.status = TaskStatus::FAILED;
            result.error_reason = failure_reason_;
            return result;
        }
        command_ = avoidance_update.setpoint;
        result.setpoint = command_;
        if (avoidance_update.status == TaskStatus::SUCCEEDED)
        {
            ROS_INFO("Yellow balloon internal avoidance completed.");
            command_ = buffer_setpoint_;
            command_.yaw = vehicle.yaw;
            beginSearch(command_, detection, now);
            result.setpoint = command_;
        }
        return result;
    }

    if (stage_ == SEARCH)
    {
        command_ = search_hold_setpoint_;
        command_.yaw = normalizeAngle(command_.yaw +
                                      config_.search_yaw_rate * dt);
        search_hold_setpoint_.yaw = command_.yaw;
        result.setpoint = command_;
        if (detectionIsFresh(detection, now) &&
            detection.sequence != last_processed_sequence_)
        {
            stage_ = ALIGN;
            aligned_detection_count_ = 0;
            last_processed_sequence_ = detection.sequence;
            ROS_INFO("First fresh yellow balloon detection acquired at pixel "
                     "(%d, %d), area_ratio=%.4f. Entering ALIGN.",
                     detection.center_x, detection.center_y,
                     detection.area_ratio);
        }
        return result;
    }

    if (stage_ == ALIGN)
    {
        if (!detectionIsFresh(detection, now))
        {
            ROS_WARN_THROTTLE(1.0,
                "Yellow balloon detection expired. Returning to SEARCH.");
            beginSearch(command_, detection, now);
            result.setpoint = command_;
            return result;
        }

        const double error_x = detection.center_x - config_.image_center_x;
        const double error_y = detection.center_y - config_.image_center_y;
        const double yaw_rate = clampValue(
            config_.yaw_control_sign * config_.yaw_kp * error_x,
            -config_.max_yaw_rate, config_.max_yaw_rate);
        const double vertical_speed = clampValue(
            config_.vertical_control_sign * config_.vertical_kp * error_y,
            -config_.max_vertical_speed, config_.max_vertical_speed);
        command_.x = search_hold_setpoint_.x;
        command_.y = search_hold_setpoint_.y;
        command_.yaw = normalizeAngle(command_.yaw + yaw_rate * dt);
        command_.z = clampValue(command_.z + vertical_speed * dt,
                                config_.min_altitude,
                                config_.max_altitude);
        result.setpoint = command_;

        if (detection.sequence != last_processed_sequence_)
        {
            last_processed_sequence_ = detection.sequence;
            if (detectionIsCentered(detection))
            {
                ++aligned_detection_count_;
            }
            else
            {
                aligned_detection_count_ = 0;
            }
        }

        if (aligned_detection_count_ >= config_.alignment_count_required)
        {
            locked_yaw_ = command_.yaw;
            locked_altitude_ = command_.z;
            approach_start_ = vehicle;
            approach_progress_ = 0.0;
            command_ = DesiredSetpoint(
                vehicle.x, vehicle.y, locked_altitude_, locked_yaw_);
            stage_start_time_ = now;
            stage_ = APPROACH;
            result.setpoint = command_;
            ROS_INFO("Yellow balloon alignment succeeded. Locked yaw=%.3f rad, "
                     "z=%.3f m.", locked_yaw_, locked_altitude_);
        }
        return result;
    }

    if (stage_ == APPROACH)
    {
        if (!detectionIsFresh(detection, now))
        {
            ROS_WARN("Yellow balloon detection lost during APPROACH. "
                     "Holding the last safe setpoint and returning to SEARCH.");
            beginSearch(command_, detection, now);
            result.setpoint = command_;
            return result;
        }

        if (detection.area_ratio >= config_.area_ratio_threshold)
        {
            contact_start_ = vehicle;
            contact_goal_ = DesiredSetpoint(
                contact_start_.x + config_.contact_distance *
                    std::cos(locked_yaw_),
                contact_start_.y + config_.contact_distance *
                    std::sin(locked_yaw_),
                locked_altitude_, locked_yaw_);
            command_ = DesiredSetpoint(
                contact_start_.x, contact_start_.y,
                locked_altitude_, locked_yaw_);
            contact_progress_ = 0.0;
            stage_start_time_ = now;
            stage_ = CONTACT_ACTION;
            result.setpoint = command_;
            ROS_INFO("Yellow balloon area_ratio %.4f reached threshold %.4f.",
                     detection.area_ratio, config_.area_ratio_threshold);
            ROS_INFO("Yellow balloon final contact action started: "
                     "goal=(%.3f, %.3f), distance=%.3f m, speed=%.3f m/s.",
                     contact_goal_.x, contact_goal_.y,
                     config_.contact_distance, config_.contact_speed);
            return result;
        }

        const double elapsed = std::max(
            0.0, (now - stage_start_time_).toSec());
        const double elapsed_progress = config_.approach_speed * elapsed;
        approach_progress_ = std::min(
            elapsed_progress,
            approach_progress_ + config_.approach_speed * dt);
        if (elapsed > config_.approach_timeout ||
            approach_progress_ >= config_.max_approach_distance)
        {
            fail("yellow balloon approach exceeded its time or distance limit");
            result.status = TaskStatus::FAILED;
            result.error_reason = failure_reason_;
            return result;
        }
        command_ = DesiredSetpoint(
            approach_start_.x + approach_progress_ * std::cos(locked_yaw_),
            approach_start_.y + approach_progress_ * std::sin(locked_yaw_),
            locked_altitude_, locked_yaw_);
        result.setpoint = command_;
        return result;
    }

    if (stage_ == CONTACT_ACTION)
    {
        const double elapsed = std::max(
            0.0, (now - stage_start_time_).toSec());
        if (elapsed > config_.contact_timeout)
        {
            fail("yellow balloon contact action timed out");
            result.status = TaskStatus::FAILED;
            result.error_reason = failure_reason_;
            return result;
        }

        const double elapsed_progress = std::min(
            config_.contact_distance, config_.contact_speed * elapsed);
        contact_progress_ = std::min(
            elapsed_progress,
            contact_progress_ + config_.contact_speed * dt);
        command_ = DesiredSetpoint(
            contact_start_.x + contact_progress_ * std::cos(locked_yaw_),
            contact_start_.y + contact_progress_ * std::sin(locked_yaw_),
            locked_altitude_, locked_yaw_);
        result.setpoint = command_;
        if (distance3D(vehicle.x, vehicle.y, vehicle.z,
                       contact_goal_.x, contact_goal_.y,
                       contact_goal_.z) <=
            config_.contact_arrive_tolerance)
        {
            command_ = contact_goal_;
            result.setpoint = command_;
            stage_ = FINISHED;
            result.status = TaskStatus::SUCCEEDED;
            ROS_INFO("Yellow balloon task completed at final contact goal.");
        }
    }
    return result;
}

void YellowBalloonTask::reset()
{
    approach_avoidance_.reset();
    stage_ = IDLE;
    start_hold_pending_ = false;
    aligned_detection_count_ = 0;
    last_processed_sequence_ = 0;
    command_ = DesiredSetpoint();
    buffer_setpoint_ = DesiredSetpoint();
    search_hold_setpoint_ = DesiredSetpoint();
    approach_start_ = VehicleState();
    contact_start_ = VehicleState();
    contact_goal_ = DesiredSetpoint();
    locked_yaw_ = 0.0;
    locked_altitude_ = 0.0;
    approach_progress_ = 0.0;
    contact_progress_ = 0.0;
    last_update_time_ = ros::Time(0);
    stage_start_time_ = ros::Time(0);
    failure_reason_.clear();
}

const char* YellowBalloonTask::stageName() const
{
    switch (stage_)
    {
        case IDLE: return "IDLE";
        case MOVE_TO_BUFFER: return "MOVE_TO_BUFFER";
        case SEARCH: return "SEARCH";
        case ALIGN: return "ALIGN";
        case APPROACH: return "APPROACH";
        case CONTACT_ACTION: return "CONTACT_ACTION";
        case FINISHED: return "FINISHED";
        case FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

}  // namespace offboard_test
