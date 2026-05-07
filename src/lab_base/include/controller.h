#pragma once

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "state_manager.h"
#include "transformation.h"

struct PIDControllerGains {
    double kp;
    double kd;
};
struct TargetDistance {
    double angular;
    double waypoint;
};
struct GoalDistance {
    double threshold;
};

class Controller{
public:
    Controller(StateManager& sm) : state_manager(sm) {}
    TurnResult turn_controller(
        Eigen::Vector3d& current_angle,
        Stamped3DVector& target_position,
        Stamped3DVector& current_position,
        double sample_time,
        PositionError& previous_position_error);
    geometry_msgs::msg::Twist dd_PD_controller(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_position_error,
        PositionError& previous_angle_error
        );
    
    geometry_msgs::msg::Twist dd_PD_precision_controller(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_position_error,
        PositionError& previous_angle_error
        );
    bool simple_distance_test(const Stamped3DVector& current_position, const Stamped3DVector& target_position);
    double euclidean_distance(const Stamped3DVector& current_position, const Stamped3DVector& target_position);
    void setGains(const PIDControllerGains& turning_gains, const PIDControllerGains& lin_gains, const PIDControllerGains& ang_gains, const PIDControllerGains& lin_precision_gains);

    geometry_msgs::msg::Twist dd_PD_controller_2(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_position_error,
        PositionError& previous_angle_error,
        Stamped3DVector& global_velocity
        );
    geometry_msgs::msg::Twist od_PD_precision_controller(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_position_error,
        PositionError& previous_angle_error
        );

private:
    StateManager& state_manager;
    Transformation transformation;
    PIDControllerGains pd_angular_gains{1.0, 0.1}; // Default gains, can be set using setGains
    PIDControllerGains pd_linear_gains{1.0, 0.1}; // Default gains, can be set using setGains
    PIDControllerGains pd_linear_precision_gains{1.0, 0.5}; // Default gains, can be set using setGains
    PIDControllerGains pd_turning_gains{1.0, 0.1}; // Default gains, can be set using setGains
};