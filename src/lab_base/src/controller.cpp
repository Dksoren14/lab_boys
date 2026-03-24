#include "controller.h"


TurnResult Controller::turn_controller(Eigen::Vector3d& target_vector,
    Eigen::Vector3d& current_angle,
    double sample_time,
    PositionError& previous_position_error){
    
    double angle_error = target_vector.z() - current_angle.x();

    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    double angle_error_d = 0.0;
    if (sample_time > 0.0) {
        angle_error_d = (angle_error - previous_position_error.Z.error) / sample_time;
    }
    geometry_msgs::msg::Twist output_velocity;
    output_velocity.angular.z = 1.0 * angle_error + 0.1 * angle_error_d;
    previous_position_error.Z.error = angle_error;
    
    return TurnResult{output_velocity, angle_error};
}

geometry_msgs::msg::Twist Controller::simple_controller(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Eigen::Vector3d& target_vector,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_position_error
        )
    {
    // Position error (just X for now)
    double position_error_x = target_position.x() - current_position.x();
    double angle_error = target_vector.z() - current_angle.x();

    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // Derivative (just X for now)
    double position_error_x_d = 0.0;
    double angle_error_d = 0.0;
    if (sample_time > 0.0) {
        position_error_x_d = (position_error_x - previous_position_error.X.error) / sample_time;
        angle_error_d = (angle_error - previous_position_error.Z.error) / sample_time;
    }

    // PD output
    geometry_msgs::msg::Twist output_velocity;
    output_velocity.linear.x = 1.0 * position_error_x + 0.1 * position_error_x_d;
    output_velocity.angular.z = 1.0 * angle_error + 0.1 * angle_error_d;

    // Save error for next tick
    previous_position_error.X.error = position_error_x;
    previous_position_error.Z.error = angle_error;

    output_velocity.linear.x = std::clamp(output_velocity.linear.x, -5.0, 5.0); 


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