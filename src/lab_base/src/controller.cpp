#include "controller.h"



geometry_msgs::msg::Twist Controller::simple_controller(const Stamped3DVector& current_position, 
    const Stamped3DVector& target_position, 
    const Stamped3DVector& current_velocity, 
    double sample_time,
    PositionError previous_position_error) 
{
    // Position error (just X for now)
    double position_error_x = target_position.x() - current_position.x();

    // Derivative (just X for now)
    double position_error_x_d = 0.0;
    if (sample_time > 0.0) {
        position_error_x_d = (position_error_x - previous_position_error.X.error) / sample_time;
    }

    // PD output
    geometry_msgs::msg::Twist output_velocity;
    output_velocity.linear.x = 1.0 * position_error_x + 0.1 * position_error_x_d;

    // Save error for next tick
    previous_position_error.X.error = position_error_x;

    return output_velocity;
}

bool Controller::simple_distance_test(const Stamped3DVector& current_position, const Stamped3DVector& target_position) {
    //double distance = (target_position.vector() - current_position.vector()).norm();
    //RCLCPP_INFO(rclcpp::get_logger("Controller"), "Distance to target: %.2f", distance);
    //RCLCPP_INFO(rclcpp::get_logger("Controller"), "Current position: x=%.2f, y=%.2f, z=%.2f", current_position.x(), current_position.y(), current_position.z());
    float distance = (target_position.vector().x() - current_position.vector().x());
    //RCLCPP_INFO(rclcpp::get_logger("Controller"), "Distance to target: %.2f", distance);
    return distance < 0.05; // Example threshold
}