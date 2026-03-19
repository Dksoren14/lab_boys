#include "controller.h"



geometry_msgs::msg::Twist Controller::simple_controller(const Stamped3DVector& current_position, 
    const Stamped3DVector& target_position, 
    const Stamped3DVector& current_velocity) {
   
    geometry_msgs::msg::Twist target; // Example target velocity
    target.linear.x = 2.0;
    target.linear.y = 0.0;
    target.linear.z = 0.0;
    geometry_msgs::msg::Twist error;
    error.linear.x = target.linear.x - current_velocity.x();
    error.linear.y = target.linear.y - current_velocity.y();
    error.linear.z = target.linear.z - current_velocity.z();


    previous_error += error;
    double k = 1.0;

    geometry_msgs::msg::Twist output_velocity;
    output_velocity.linear.x = k * error.linear.x;
    output_velocity.linear.y = k * error.linear.y;
    output_velocity.linear.z = k * error.linear.z;
    RCLCPP_INFO(rclcpp::get_logger("Controller"), "Output velocity: x=%.2f, y=%.2f, z=%.2f", output_velocity.linear.x, output_velocity.linear.y, output_velocity.linear.z);
    
    


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