#pragma once

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


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
    PIDError dX;
};

struct TurnResult {
    geometry_msgs::msg::Twist cmd;
    double angle_error;
};


class StateManager {
public:
    void setLocalPosition(const Stamped3DVector& position);
    Stamped3DVector getLocalPosition();
    void setGlobalBasePosition(const Stamped3DVector& position);
    Stamped3DVector getGlobalBasePosition();
    void setGlobalBaseOrientation(const Eigen::Quaterniond& orientation);
    Eigen::Quaterniond getGlobalBaseOrientation();
    void setGlobalBaseVelocity(const Stamped3DVector& velocity);
    Stamped3DVector getGlobalBaseVelocity();
    void setLocalVelocity(const Stamped3DVector& velocity);
    Stamped3DVector getLocalVelocity();
    void setTargetPosition(const Stamped3DVector& target_position);
    Stamped3DVector getTargetPosition();
    void setControlMode(const int& mode);
    int getControlMode();
    void setPositionError(const PositionError& error);
    PositionError getPositionError();
    void setGoalPosition(const Stamped3DVector& goal_position);
    Stamped3DVector getGoalPosition();
    void setPath(std::vector<geometry_msgs::msg::PoseStamped>& path);
    std::vector<geometry_msgs::msg::PoseStamped> getPath();
    void setArucoPose(const Stamped3DVector& aruco_pose);
    Stamped3DVector getArucoPose();
    void setArucoOrientation(const Eigen::Quaterniond& aruco_orientation);
    Eigen::Quaterniond getArucoOrientation();
    void setTrueGoalPosition(const Stamped3DVector& true_goal_position);
    Stamped3DVector getTrueGoalPosition();
    


private:
    std::mutex local_position_mutex;
    Stamped3DVector local_position;
    std::mutex global_base_position_mutex;
    Stamped3DVector global_base_position;
    std::mutex global_base_orientation_mutex;
    Eigen::Quaterniond global_base_orientation;
    std::mutex global_base_velocity_mutex;
    Stamped3DVector global_base_velocity;
    std::mutex local_velocity_mutex;
    Stamped3DVector local_velocity;
    std::mutex target_position_mutex;
    Stamped3DVector target_position;
    std::mutex control_mode_mutex;
    std::mutex position_error_mutex;
    PositionError position_error;
    std::mutex goal_position_mutex;
    Stamped3DVector goal_position;
    std::mutex path_mutex;
    std::vector<geometry_msgs::msg::PoseStamped> path;
    std::mutex aruco_pose_mutex;
    Stamped3DVector aruco_pose;
    std::mutex aruco_orientation_mutex;
    Eigen::Quaterniond aruco_orientation;
    std::mutex true_goal_position_mutex;
    Stamped3DVector true_goal_position;

    int control_mode = 0; 
};