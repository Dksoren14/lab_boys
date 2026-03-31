#pragma once

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "state_manager.h"


class Transformation {
public:   
    Eigen::Vector3d global_to_local(
        const Stamped3DVector& target_vector, 
        const Stamped3DVector& current_position, 
        const Eigen::Vector3d& current_orientation);
    double calculate_angle_to_target(
        const Stamped3DVector& target_vector,
        const Stamped3DVector& current_position, 
        const Eigen::Vector3d& current_orientation);
    Eigen::Vector3d global_to_local_error(const Stamped3DVector& current_position,
        Eigen::Vector3d& global_error,
        const Eigen::Vector3d& current_orientation);
    Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) const;
    Eigen::Quaterniond euler_to_quaternion(double roll, double pitch, double yaw) const;
    
    double unwrapAngle(double angle, double max, double min) const;

private:

};

