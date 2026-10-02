#ifndef OFFBOARD_CIRCLE_MODIFIED_SATELLITE_TASK_H
#define OFFBOARD_CIRCLE_MODIFIED_SATELLITE_TASK_H

#include <string>

#include <offboard_circle_modified/mission_types.h>

namespace offboard_test
{

struct SatelliteConfig
{
    SatelliteConfig();

    double center_x;
    double center_y;
    double altitude;
    double image_center_x;
    double kp_fast;
    double kp_slow;
    double kd;
    double aspect_threshold;
    double delta_omega;
    double search_radius;
    double orbit_radius;
    double omega_base;
    double orbit_radius_tolerance;
    double orbit_duration;
    double detection_timeout;
    int detection_count_required;
};

class SatelliteTask
{
public:
    SatelliteTask();

    bool configure(const SatelliteConfig& config, std::string& error);
    bool start(const VehicleState& vehicle, std::string& error);
    TaskUpdate update(const VehicleState& vehicle,
                      const DetectionSnapshot& detection,
                      const ros::Time& now);
    void reset();
    void handlePlay4Result(bool success, const ros::Time& now);

    const char* stageName() const;

private:
    enum Stage
    {
        IDLE,
        SEARCH,
        APPROACH_ORBIT,
        WAIT_PLAY4,
        ORBIT,
        DEPART,
        COMPLETE,
        FAILED
    };

    DesiredSetpoint circleSetpoint(const VehicleState& vehicle,
                                   double radius) const;
    bool detectionIsFresh(const DetectionSnapshot& detection,
                          const ros::Time& now) const;

    SatelliteConfig config_;
    Stage stage_;
    double theta_;
    double omega_circle_;
    double last_error_x_;
    int detection_count_;
    bool play4_pending_;
    ros::Time orbit_start_time_;
    ros::Time last_update_time_;
};

}  // namespace offboard_test

#endif
