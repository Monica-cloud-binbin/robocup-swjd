#ifndef OFFBOARD_CIRCLE_MODIFIED_YELLOW_BALLOON_TASK_H
#define OFFBOARD_CIRCLE_MODIFIED_YELLOW_BALLOON_TASK_H

#include <cstdint>
#include <string>

#include <offboard_circle_modified/mission_types.h>

namespace offboard_test
{

struct YellowBalloonConfig
{
    YellowBalloonConfig();

    DesiredSetpoint staging_setpoint;
    double arrive_tolerance;
    double image_center_x;
    double image_center_y;
    double pixel_tolerance;
    double detection_timeout;
    int detection_count_required;
};

class YellowBalloonTask
{
public:
    enum Stage
    {
        IDLE,
        MOVE_TO_STAGING,
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

    YellowBalloonConfig config_;
    Stage stage_;
    int centered_detection_count_;
    std::uint64_t last_processed_sequence_;
};

}  // namespace offboard_test

#endif
