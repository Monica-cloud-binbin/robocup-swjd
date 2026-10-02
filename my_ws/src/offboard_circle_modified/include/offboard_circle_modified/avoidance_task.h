#ifndef OFFBOARD_CIRCLE_MODIFIED_AVOIDANCE_TASK_H
#define OFFBOARD_CIRCLE_MODIFIED_AVOIDANCE_TASK_H

#include <string>

#include <offboard_circle_modified/mission_types.h>

namespace offboard_test
{

struct AvoidanceConfig
{
    AvoidanceConfig();

    double obstacle_x;
    double obstacle_y;
    double safety_radius;
    double orbit_angle_deg;
    double orbit_angular_speed;
    int orbit_direction;
    double arrive_tolerance;
    double orbit_tracking_tolerance;
};

class AvoidanceTask
{
public:
    AvoidanceTask();

    bool configure(const AvoidanceConfig& config, std::string& error);
    bool start(const VehicleState& vehicle,
               const DesiredSetpoint& target,
               int direction,
               std::string& error);
    TaskUpdate update(const VehicleState& vehicle, const ros::Time& now);
    void reset();

    const char* stageName() const;

private:
    enum Stage
    {
        IDLE,
        APPROACH,
        ORBIT,
        TO_TARGET,
        COMPLETE,
        FAILED
    };

    bool directPathNeedsAvoidance(double start_x, double start_y,
                                  double goal_x, double goal_y,
                                  double& minimum_distance) const;

    AvoidanceConfig config_;
    Stage stage_;
    DesiredSetpoint target_;
    double approach_x_;
    double approach_y_;
    double orbit_theta_;
    double orbit_start_theta_;
    double orbit_target_theta_;
    double path_min_distance_;
    int direction_;
    ros::Time last_update_time_;
};

}  // namespace offboard_test

#endif
