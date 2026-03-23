#pragma once

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>

struct Stamped3DVector {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    Eigen::Vector3d data = Eigen::Vector3d::Zero();

    // Default constructor
    Stamped3DVector() = default;

    // Constructor with timestamp and components (added for controlMode and manualAidedMode)
    Stamped3DVector(const rclcpp::Time& ts, double x, double y, double z)
        : timestamp(ts), data(x, y, z) {}

    // Get individual components
    double x() const { return data.x(); }
    double y() const { return data.y(); }
    double z() const { return data.z(); }

    // Set individual components
    void setX(double value) { data.x() = value; }
    void setY(double value) { data.y() = value; }
    void setZ(double value) { data.z() = value; }

    // Get the full vector
    const Eigen::Vector3d& vector() const { return data; }
    Eigen::Vector3d& vector() { return data; }

    // Get and set timestamp
    rclcpp::Time getTime() const { return timestamp; }
    void setTime(const rclcpp::Time& new_time) { timestamp = new_time; }
};

struct PIDError {
    double error;
    double error_d;
    double error_integral;
};

struct PositionError {
    PIDError X;
    PIDError Y;
    PIDError Z;
    PIDError Theta;
};


class StateManager {
public:
    void setLocalPosition(const Stamped3DVector& position);
    Stamped3DVector getLocalPosition();
    void setGlobalBasePosition(const Stamped3DVector& position);
    Stamped3DVector getGlobalBasePosition();
    void setGlobalBaseOrientation(const Stamped3DVector& orientation);
    Stamped3DVector getGlobalBaseOrientation();
    void setLocalVelocity(const Stamped3DVector& velocity);
    Stamped3DVector getLocalVelocity();
    void setTargetPosition(const Stamped3DVector& target_position);
    Stamped3DVector getTargetPosition();
    void setControlMode(const int& mode);
    int getControlMode();
    void setPositionError(const PositionError& error);
    PositionError getPositionError();
    


private:
    std::mutex local_position_mutex;
    Stamped3DVector local_position;
    std::mutex global_base_position_mutex;
    Stamped3DVector global_base_position;
    std::mutex global_base_orientation_mutex;
    Stamped3DVector global_base_orientation;
    std::mutex local_velocity_mutex;
    Stamped3DVector local_velocity;
    std::mutex target_position_mutex;
    Stamped3DVector target_position;
    std::mutex control_mode_mutex;
    std::mutex position_error_mutex;
    PositionError position_error;
    int control_mode = 0; 
};