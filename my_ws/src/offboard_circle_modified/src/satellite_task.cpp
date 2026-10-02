#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include <offboard_circle_modified/satellite_task.h>

namespace offboard_test
{

SatelliteConfig::SatelliteConfig()
    : center_x(0.0), center_y(0.0), altitude(1.0),
      image_center_x(320.0), kp_fast(0.00035), kp_slow(0.00020),
      kd(0.0003), aspect_threshold(1.1), delta_omega(0.05),
      search_radius(1.4), orbit_radius(0.8), omega_base(0.116),
      orbit_radius_tolerance(0.10), orbit_duration(10.0),
      detection_timeout(0.5), detection_count_required(6)
{
}

SatelliteTask::SatelliteTask()
{
    reset();
}

bool SatelliteTask::configure(const SatelliteConfig& config,
                              std::string& error)
{
    if (config.altitude <= 0.0 || config.search_radius <= 0.0 ||
        config.orbit_radius <= 0.0 || config.omega_base <= 0.0)
    {
        error = "satellite altitude, radii and omega must be positive";
        return false;
    }
    if (config.orbit_radius_tolerance <= 0.0 ||
        config.orbit_duration <= 0.0 || config.detection_timeout <= 0.0 ||
        config.detection_count_required <= 0)
    {
        error = "satellite timing/tolerance/count parameters are invalid";
        return false;
    }
    config_ = config;
    error.clear();
    return true;
}

bool SatelliteTask::start(const VehicleState& vehicle, std::string& error)
{
    reset();
    if (!vehicle.valid)
    {
        error = "cannot start satellite task without valid odometry";
        stage_ = FAILED;
        return false;
    }

    theta_ = std::atan2(vehicle.y - config_.center_y,
                        vehicle.x - config_.center_x);
    omega_circle_ = -config_.omega_base / 2.0;
    stage_ = SEARCH;
    last_update_time_ = ros::Time::now();
    error.clear();
    ROS_INFO("Satellite task started: center=(%.3f, %.3f), search R=%.3f, "
             "orbit R=%.3f.", config_.center_x, config_.center_y,
             config_.search_radius, config_.orbit_radius);
    return true;
}

bool SatelliteTask::detectionIsFresh(
    const DetectionSnapshot& detection, const ros::Time& now) const
{
    return detection.valid && !detection.message_time.isZero() &&
           now - detection.message_time < ros::Duration(config_.detection_timeout);
}

DesiredSetpoint SatelliteTask::circleSetpoint(
    const VehicleState& vehicle, double radius) const
{
    return DesiredSetpoint(
        config_.center_x + radius * std::cos(theta_),
        config_.center_y + radius * std::sin(theta_),
        config_.altitude,
        std::atan2(config_.center_y - vehicle.y,
                   config_.center_x - vehicle.x));
}

TaskUpdate SatelliteTask::update(const VehicleState& vehicle,
                                 const DetectionSnapshot& detection,
                                 const ros::Time& now)
{
    TaskUpdate result;
    result.status = TaskStatus::RUNNING;
    if (stage_ == IDLE)
    {
        result.status = TaskStatus::IDLE;
        result.setpoint = DesiredSetpoint(vehicle.x, vehicle.y,
                                          vehicle.z, vehicle.yaw);
        return result;
    }
    if (stage_ == FAILED)
    {
        result.status = TaskStatus::FAILED;
        result.error_reason = "satellite task is in FAILED state";
        return result;
    }
    if (!vehicle.valid)
    {
        stage_ = FAILED;
        result.status = TaskStatus::FAILED;
        result.error_reason = "odometry became invalid during satellite task";
        return result;
    }
    if (stage_ == COMPLETE)
    {
        result.status = TaskStatus::SUCCEEDED;
        result.setpoint = circleSetpoint(vehicle, config_.orbit_radius);
        return result;
    }

    last_update_time_ = now;

    if (stage_ == SEARCH)
    {
        const bool detected = detectionIsFresh(detection, now) &&
            detection.center_x >= 245 && detection.center_x <= 370;
        if (detected)
        {
            ++detection_count_;
        }
        else
        {
            detection_count_ = 0;
        }

        if (detection_count_ >= config_.detection_count_required)
        {
            stage_ = APPROACH_ORBIT;
            omega_circle_ = config_.omega_base;
            ROS_INFO("Satellite visual confirmation complete. Moving to orbit radius.");
        }
        else
        {
            omega_circle_ = -config_.omega_base / 2.0;
        }
    }

    if (stage_ == APPROACH_ORBIT || stage_ == WAIT_PLAY4 || stage_ == ORBIT)
    {
        if (detectionIsFresh(detection, now))
        {
            double error_x = detection.center_x - config_.image_center_x;
            error_x = std::max(-200.0, std::min(200.0, error_x));
            const double kp = error_x > 0.0 ? config_.kp_fast : config_.kp_slow;
            const double d_error_x = (error_x - last_error_x_) * 20.0;
            last_error_x_ = error_x;
            omega_circle_ = config_.omega_base + kp * error_x +
                            config_.kd * d_error_x;
            if (detection.aspect_ratio > config_.aspect_threshold)
            {
                omega_circle_ += detection.center_x < config_.image_center_x
                    ? -config_.delta_omega : config_.delta_omega;
            }
        }
        else
        {
            omega_circle_ = config_.omega_base;
            last_error_x_ = 0.0;
        }
    }

    if (stage_ == SEARCH)
    {
        omega_circle_ = std::max(-0.25, std::min(-0.02, omega_circle_));
        theta_ += omega_circle_ * (1.0 / 20.0);
        result.setpoint = circleSetpoint(vehicle, config_.search_radius);
        return result;
    }

    omega_circle_ = std::max(0.02, std::min(0.25, omega_circle_));
    theta_ += omega_circle_ * (1.0 / 20.0);
    result.setpoint = circleSetpoint(vehicle, config_.orbit_radius);

    if (stage_ == APPROACH_ORBIT)
    {
        const double current_radius = distance2D(
            vehicle.x, vehicle.y, config_.center_x, config_.center_y);
        if (std::fabs(current_radius - config_.orbit_radius) <
            config_.orbit_radius_tolerance)
        {
            stage_ = WAIT_PLAY4;
            play4_pending_ = true;
        }
    }

    if (stage_ == WAIT_PLAY4 && play4_pending_)
    {
        result.request_play4 = true;
    }
    else if (stage_ == ORBIT &&
             now - orbit_start_time_ > ros::Duration(config_.orbit_duration))
    {
        // The formal controller has no separate departure trajectory. DEPART
        // is therefore a one-cycle handoff at the last safe orbit setpoint.
        stage_ = DEPART;
        ROS_INFO("Satellite timed orbit finished. Handing off at the final orbit setpoint.");
    }
    else if (stage_ == DEPART)
    {
        stage_ = COMPLETE;
        result.status = TaskStatus::SUCCEEDED;
    }
    return result;
}

void SatelliteTask::handlePlay4Result(bool success, const ros::Time& now)
{
    if (stage_ != WAIT_PLAY4)
    {
        return;
    }
    play4_pending_ = false;
    if (success)
    {
        orbit_start_time_ = now;
        stage_ = ORBIT;
        ROS_INFO("play4 service succeeded. Start satellite orbit timer.");
    }
    else
    {
        play4_pending_ = true;
        ROS_WARN_THROTTLE(1.0,
            "play4 service failed. Waiting at satellite orbit radius.");
    }
}

void SatelliteTask::reset()
{
    stage_ = IDLE;
    theta_ = 0.0;
    omega_circle_ = 0.0;
    last_error_x_ = 0.0;
    detection_count_ = 0;
    play4_pending_ = false;
    orbit_start_time_ = ros::Time(0);
    last_update_time_ = ros::Time(0);
}

const char* SatelliteTask::stageName() const
{
    switch (stage_)
    {
        case IDLE: return "IDLE";
        case SEARCH: return "SEARCH";
        case APPROACH_ORBIT: return "APPROACH_ORBIT";
        case WAIT_PLAY4: return "WAIT_PLAY4";
        case ORBIT: return "ORBIT";
        case DEPART: return "DEPART";
        case COMPLETE: return "COMPLETE";
        case FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

}  // namespace offboard_test
