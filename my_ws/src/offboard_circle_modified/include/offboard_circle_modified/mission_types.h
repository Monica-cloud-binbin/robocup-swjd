#ifndef OFFBOARD_CIRCLE_MODIFIED_MISSION_TYPES_H
#define OFFBOARD_CIRCLE_MODIFIED_MISSION_TYPES_H

#include <cmath>
#include <cstdint>
#include <string>

#include <ros/time.h>

namespace offboard_test
{

struct VehicleState
{
    VehicleState()
        : x(0.0), y(0.0), z(0.0), yaw(0.0), valid(false)
    {
    }

    double x;
    double y;
    double z;
    double yaw;
    ros::Time odometry_time;
    bool valid;
};

struct DetectionSnapshot
{
    DetectionSnapshot()
        : sequence(0), confidence(0.0), center_x(0), center_y(0),
          aspect_ratio(0.0), area_ratio(0.0), valid(false)
    {
    }

    std::string class_name;
    std::uint64_t sequence;
    ros::Time message_time;
    float confidence;
    int center_x;
    int center_y;
    float aspect_ratio;
    float area_ratio;
    bool valid;
};

struct DesiredSetpoint
{
    DesiredSetpoint()
        : x(0.0), y(0.0), z(0.0), yaw(0.0)
    {
    }

    DesiredSetpoint(double x_value, double y_value, double z_value,
                    double yaw_value)
        : x(x_value), y(y_value), z(z_value), yaw(yaw_value)
    {
    }

    double x;
    double y;
    double z;
    double yaw;
};

enum class TaskStatus
{
    IDLE,
    RUNNING,
    SUCCEEDED,
    FAILED
};

struct TaskUpdate
{
    TaskUpdate()
        : status(TaskStatus::IDLE), request_play4(false)
    {
    }

    TaskStatus status;
    DesiredSetpoint setpoint;
    std::string error_reason;
    bool request_play4;
};

inline double distance2D(double x1, double y1, double x2, double y2)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

inline double distance3D(double x1, double y1, double z1,
                         double x2, double y2, double z2)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    const double dz = z1 - z2;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace offboard_test

#endif
