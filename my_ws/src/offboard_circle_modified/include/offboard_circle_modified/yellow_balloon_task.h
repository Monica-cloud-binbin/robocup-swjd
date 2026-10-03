#ifndef OFFBOARD_CIRCLE_MODIFIED_YELLOW_BALLOON_TASK_H
#define OFFBOARD_CIRCLE_MODIFIED_YELLOW_BALLOON_TASK_H

#include <cstdint>
#include <string>

#include <offboard_circle_modified/avoidance_task.h>
#include <offboard_circle_modified/mission_types.h>

namespace offboard_test
{

struct YellowBalloonConfig
{
    YellowBalloonConfig();

    // Kept for compatibility with the protected takeoff setup. This setpoint
    // stores the pre-calibrated yellow-balloon center in local odom coordinates.
    DesiredSetpoint staging_setpoint;
    AvoidanceConfig approach_avoidance;
    double buffer_distance;
    double search_yaw_rate;
    double image_center_x;
    double image_center_y;
    double alignment_tolerance_x;
    double alignment_tolerance_y;
    int alignment_count_required;
    double yaw_kp;
    double yaw_control_sign;
    double max_yaw_rate;
    double vertical_kp;
    double vertical_control_sign;
    double max_vertical_speed;
    double min_altitude;
    double max_altitude;
    double target_x_offset;
    double target_y_offset;
    double target_altitude_offset;
    double detection_timeout;
    double approach_speed;
    double area_ratio_threshold;
    double max_approach_distance;
    double approach_timeout;
    double contact_speed;
    double contact_distance;
    double contact_arrive_tolerance;
    double contact_timeout;
};

class YellowBalloonTask
{
public:
    enum Stage
    {
        IDLE,
        MOVE_TO_BUFFER,
        SEARCH,
        ALIGN,
        APPROACH,
        CONTACT_ACTION,
        FINISHED,
        FAILED
    };

    YellowBalloonTask();

    bool configure(const YellowBalloonConfig& config, std::string& error);
    bool start(const VehicleState& vehicle, std::string& error);
    TaskUpdate update(const VehicleState& vehicle,
                      const DetectionSnapshot& detection,
                      const ros::Time& now);
    void reset();

    const char* stageName() const;

private:
    bool detectionIsFresh(const DetectionSnapshot& detection,
                          const ros::Time& now) const;
    bool detectionIsCentered(const DetectionSnapshot& detection) const;
    void fail(const std::string& reason);
    double updateDt(const ros::Time& now);
    void beginSearch(const DesiredSetpoint& hold_setpoint,
                     const DetectionSnapshot& detection,
                     const ros::Time& now);

    YellowBalloonConfig config_;
    AvoidanceTask approach_avoidance_;
    Stage stage_;
    bool configured_;
    bool start_hold_pending_;
    int aligned_detection_count_;
    std::uint64_t last_processed_sequence_;
    DesiredSetpoint command_;
    DesiredSetpoint buffer_setpoint_;
    DesiredSetpoint search_hold_setpoint_;
    VehicleState approach_start_;
    VehicleState contact_start_;
    DesiredSetpoint contact_goal_;
    double locked_yaw_;
    double locked_altitude_;
    double approach_progress_;
    double contact_progress_;
    ros::Time last_update_time_;
    ros::Time stage_start_time_;
    std::string failure_reason_;
};

}  // namespace offboard_test

#endif
