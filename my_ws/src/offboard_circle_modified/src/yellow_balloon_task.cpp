#include <cmath>

#include <ros/ros.h>

#include <offboard_circle_modified/yellow_balloon_task.h>

namespace offboard_test
{

YellowBalloonConfig::YellowBalloonConfig()
    : arrive_tolerance(0.12), image_center_x(320.0), image_center_y(240.0),
      pixel_tolerance(60.0), detection_timeout(0.5),
      detection_count_required(5)
{
}

YellowBalloonTask::YellowBalloonTask()
{
    reset();
}

bool YellowBalloonTask::configure(const YellowBalloonConfig& config,
                                  std::string& error)
{
    if (config.arrive_tolerance <= 0.0 || config.pixel_tolerance < 0.0 ||
        config.detection_timeout <= 0.0 || config.detection_count_required <= 0)
    {
        error = "yellow balloon tolerance/timeout/count parameters are invalid";
        return false;
    }
    config_ = config;
    error.clear();
    return true;
}

bool YellowBalloonTask::start(const VehicleState& vehicle,
                              std::string& error)
{
    reset();
    if (!vehicle.valid)
    {
        error = "cannot start yellow balloon task without valid odometry";
        stage_ = FAILED;
        return false;
    }
    stage_ = MOVE_TO_STAGING;
    error.clear();
    ROS_INFO("Yellow balloon task started. Moving to/confirming staging point.");
    return true;
}

bool YellowBalloonTask::detectionIsFresh(
    const DetectionSnapshot& detection, const ros::Time& now) const
{
    return detection.valid && !detection.message_time.isZero() &&
           now - detection.message_time < ros::Duration(config_.detection_timeout);
}

bool YellowBalloonTask::detectionIsCentered(
    const DetectionSnapshot& detection) const
{
    return std::fabs(detection.center_x - config_.image_center_x) <=
               config_.pixel_tolerance &&
           std::fabs(detection.center_y - config_.image_center_y) <=
               config_.pixel_tolerance;
}

TaskUpdate YellowBalloonTask::update(
    const VehicleState& vehicle, const DetectionSnapshot& detection,
    const ros::Time& now)
{
    TaskUpdate result;
    result.setpoint = config_.staging_setpoint;

    if (stage_ == IDLE)
    {
        result.status = TaskStatus::IDLE;
        return result;
    }
    if (stage_ == FAILED)
    {
        result.status = TaskStatus::FAILED;
        result.error_reason = "yellow balloon task is in FAILED state";
        return result;
    }
    if (!vehicle.valid)
    {
        stage_ = FAILED;
        result.status = TaskStatus::FAILED;
        result.error_reason = "odometry became invalid during yellow balloon task";
        return result;
    }
    if (stage_ == FINISHED)
    {
        result.status = TaskStatus::SUCCEEDED;
        return result;
    }

    result.status = TaskStatus::RUNNING;
    if (stage_ == MOVE_TO_STAGING)
    {
        if (distance3D(vehicle.x, vehicle.y, vehicle.z,
                       config_.staging_setpoint.x,
                       config_.staging_setpoint.y,
                       config_.staging_setpoint.z) <= config_.arrive_tolerance)
        {
            stage_ = SEARCH;
            ROS_INFO("Yellow balloon staging point reached. Waiting for detection.");
        }
        return result;
    }

    if (stage_ == SEARCH)
    {
        if (detectionIsFresh(detection, now))
        {
            stage_ = ALIGN;
            centered_detection_count_ = 0;
            last_processed_sequence_ = 0;
            ROS_INFO("Fresh yellow balloon detection received. Checking alignment.");
        }
        return result;
    }

    if (stage_ == ALIGN)
    {
        if (!detectionIsFresh(detection, now))
        {
            stage_ = SEARCH;
            centered_detection_count_ = 0;
            ROS_WARN_THROTTLE(1.0,
                "Yellow balloon detection expired. Returning to SEARCH.");
            return result;
        }

        if (detection.sequence != last_processed_sequence_)
        {
            last_processed_sequence_ = detection.sequence;
            if (detectionIsCentered(detection))
            {
                ++centered_detection_count_;
            }
            else
            {
                centered_detection_count_ = 0;
            }
        }

        if (centered_detection_count_ >= config_.detection_count_required)
        {
            // APPROACH and CONTACT_ACTION remain reserved until their control
            // law and thresholds are explicitly confirmed. The current formal
            // controller treats repeated centered detections as task success.
            stage_ = FINISHED;
            result.status = TaskStatus::SUCCEEDED;
            ROS_INFO("Yellow balloon visual confirmation succeeded at pixel "
                     "(%d, %d), area_ratio=%.4f.",
                     detection.center_x, detection.center_y,
                     detection.area_ratio);
        }
        return result;
    }

    if (stage_ == APPROACH || stage_ == CONTACT_ACTION)
    {
        stage_ = FAILED;
        result.status = TaskStatus::FAILED;
        result.error_reason =
            "unconfirmed yellow balloon control stage was entered";
    }
    return result;
}

void YellowBalloonTask::reset()
{
    stage_ = IDLE;
    centered_detection_count_ = 0;
    last_processed_sequence_ = 0;
}

const char* YellowBalloonTask::stageName() const
{
    switch (stage_)
    {
        case IDLE: return "IDLE";
        case MOVE_TO_STAGING: return "MOVE_TO_STAGING";
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
