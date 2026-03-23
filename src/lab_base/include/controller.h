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


class Controller{
public:
    Controller(StateManager& sm) : state_manager(sm) {}
    geometry_msgs::msg::Twist simple_controller(const Stamped3DVector& current_position, 
        const Stamped3DVector& current_orientation,
        const Stamped3DVector& target_position, 
        double sample_time,
        PositionError previous_position_error
        );
    bool simple_distance_test(const Stamped3DVector& current_position, const Stamped3DVector& target_position);

private:
    StateManager& state_manager;
    
};