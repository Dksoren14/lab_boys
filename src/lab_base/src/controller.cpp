#include "controller.h"


TurnResult Controller::turn_controller(
    Eigen::Vector3d& current_angle,
    Stamped3DVector& target_position,
    Stamped3DVector& current_position,
    double sample_time,
    PositionError& previous_angle_error){
    double local_angle_error = transformation.calculate_angle_to_target(
        target_position,
        current_position, 
        current_angle);
    //Eigen::Vector3d local_target = transformation.global_to_local(target_position, current_position, current_angle);
    //double theta = atan2(local_target.y(), local_target.x());
    //std::cout << "Calculated angle to target: " << local_angle_error << std::endl;
    //double angle_error = local_angle_error - current_angle.z();

    while (local_angle_error > M_PI) local_angle_error -= 2.0 * M_PI;
    while (local_angle_error < -M_PI) local_angle_error += 2.0 * M_PI;

    double local_angle_error_d = 0.0;
    if (sample_time > 0.0) {
        local_angle_error_d = (local_angle_error - previous_angle_error.Z.error) / sample_time;
    }
    
    geometry_msgs::msg::Twist output_velocity;
    output_velocity.angular.z = pd_turning_gains.kp * local_angle_error + pd_turning_gains.kd * local_angle_error_d;
    previous_angle_error.Z.error = local_angle_error;
    
    return TurnResult{output_velocity, local_angle_error};
}
geometry_msgs::msg::Twist Controller::dd_PD_precision_controller(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_position_error,
        PositionError& previous_angle_error
        )
    {
    // Position error (just X for now)
    double local_angle_error = transformation.calculate_angle_to_target(
        target_position,
        current_position,
        current_angle);
    double position_error_x = target_position.x() - current_position.x();
    double position_error_y = target_position.y() - current_position.y();
    
    Eigen::Vector3d global_error(position_error_x, position_error_y, 0.0);
    
    Eigen::Vector3d local_error = transformation.global_to_local_error(current_position, global_error, current_angle);
    

    
    while (local_angle_error > M_PI) local_angle_error -= 2.0 * M_PI;
    while (local_angle_error < -M_PI) local_angle_error += 2.0 * M_PI;

    double local_angle_error_d = 0.0;
    double position_error_x_d = 0.0;
    if (sample_time > 0.0) {
        position_error_x_d = (local_error.x() - previous_position_error.X.error) / sample_time;
        local_angle_error_d = (local_angle_error - previous_angle_error.Z.error) / sample_time;
    }

    // PD output
    geometry_msgs::msg::Twist output_velocity;
    output_velocity.linear.x = pd_linear_precision_gains.kp * local_error.x() + pd_linear_precision_gains.kd * position_error_x_d;
    output_velocity.angular.z = pd_angular_gains.kp * local_angle_error + pd_angular_gains.kd * local_angle_error_d;
    
    // Save error for next tick
    previous_position_error.X.error = local_error.x();
    previous_angle_error.Z.error = local_angle_error;

    output_velocity.linear.x = std::clamp(output_velocity.linear.x, -1.1, 1.1); //Speed limit after Ridgeback Max Speed

    //output_velocity.linear.x = 0.0;
    return output_velocity;
}


geometry_msgs::msg::Twist Controller::dd_PD_controller(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_position_error,
        PositionError& previous_angle_error
        )
    {
    // Position error (just X for now)
    double local_angle_error = transformation.calculate_angle_to_target(
        target_position,
        current_position,
        current_angle);
    double position_error_x = target_position.x() - current_position.x();
    double position_error_y = target_position.y() - current_position.y();
    
    Eigen::Vector3d global_error(position_error_x, position_error_y, 0.0);
    
    Eigen::Vector3d local_error = transformation.global_to_local_error(current_position, global_error, current_angle);
    

    
    while (local_angle_error > M_PI) local_angle_error -= 2.0 * M_PI;
    while (local_angle_error < -M_PI) local_angle_error += 2.0 * M_PI;

    double local_angle_error_d = 0.0;
    double position_error_x_d = 0.0;
    if (sample_time > 0.0) {
        position_error_x_d = (local_error.x() - previous_position_error.X.error) / sample_time;
        local_angle_error_d = (local_angle_error - previous_angle_error.Z.error) / sample_time;
    }

    // PD output
    geometry_msgs::msg::Twist output_velocity;
    output_velocity.linear.x = pd_linear_gains.kp * local_error.x() + pd_linear_gains.kd * position_error_x_d;
    output_velocity.angular.z = pd_angular_gains.kp * local_angle_error + pd_angular_gains.kd * local_angle_error_d;
    
    // Save error for next tick
    previous_position_error.X.error = local_error.x();
    previous_angle_error.Z.error = local_angle_error;

    output_velocity.linear.x = std::clamp(output_velocity.linear.x, -1.1, 1.1); //Speed limit after Ridgeback Max Speed

    //output_velocity.linear.x = 0.0;
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

double Controller::euclidean_distance(const Stamped3DVector& current_position, const Stamped3DVector& target_position) {
    return (target_position.vector() - current_position.vector()).norm();
}

void Controller::setGains(const PIDControllerGains& turn_gains, const PIDControllerGains& lin_gains, const PIDControllerGains& ang_gains, const PIDControllerGains& lin_precision_gains) {
    
    pd_turning_gains = turn_gains;
    pd_linear_gains = lin_gains;
    pd_angular_gains = ang_gains;
    pd_linear_precision_gains = lin_precision_gains;
}


geometry_msgs::msg::Twist Controller::dd_PD_controller_2(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_velocity_error,
        PositionError& previous_angle_error,
        Stamped3DVector& global_velocity
        ) // Second version: regulates orientation while maintaining constant linear velocity. (proposed by Jakob S)
    {

    double local_angle_error = transformation.calculate_angle_to_target(
        target_position,
        current_position,
        current_angle);
    
    double velocity_error = 1.1 - global_velocity.x(); // Assuming we want to maintain a velocity of 0.3 m/s towards the target

    while (local_angle_error > M_PI) local_angle_error -= 2.0 * M_PI;
    while (local_angle_error < -M_PI) local_angle_error += 2.0 * M_PI;

    double local_angle_error_d = 0.0;
    double velocity_error_d = 0.0;
    std::cout << "Previous velocity error BEFORE: " << previous_velocity_error.dX.error << std::endl;
    std::cout << "Sample time: " << sample_time << std::endl;
    if (sample_time > 0.0) {
        local_angle_error_d = (local_angle_error - previous_angle_error.Z.error) / sample_time;
        velocity_error_d = (velocity_error - previous_velocity_error.dX.error) / sample_time;
    }
    // PD output
    geometry_msgs::msg::Twist output_velocity;
    output_velocity.angular.z = pd_angular_gains.kp * local_angle_error + pd_angular_gains.kd * local_angle_error_d;
    output_velocity.linear.x = pd_linear_gains.kp * velocity_error + pd_linear_gains.kd * velocity_error_d;
 
    // Save error for next tick
    previous_angle_error.Z.error = local_angle_error;
    previous_velocity_error.dX.error = velocity_error;

    output_velocity.linear.x = std::clamp(output_velocity.linear.x, -1.1, 1.1); //Speed limit after Ridgeback Max Speed

    //output_velocity.linear.x = 0.0;
    return output_velocity;
}


geometry_msgs::msg::Twist Controller::od_PD_precision_controller(const Stamped3DVector& current_position, 
        Eigen::Vector3d& current_angle,
        Stamped3DVector&  target_position,
        double sample_time,
        PositionError& previous_position_error,
        PositionError& previous_angle_error
        ) {
            geometry_msgs::msg::Twist output_velocity;

             double local_angle_error = transformation.calculate_angle_to_target(
                                                        target_position,
                                                        current_position,
                                                        current_angle);
            double position_error_x = target_position.x() - current_position.x();
            double position_error_y = target_position.y() - current_position.y();
            Eigen::Vector3d global_error(position_error_x, position_error_y, 0.0);
    
             Eigen::Vector3d local_error = transformation.global_to_local_error(current_position, global_error, current_angle);



             while (local_angle_error > M_PI) local_angle_error -= 2.0 * M_PI;
             while (local_angle_error < -M_PI) local_angle_error += 2.0 * M_PI;

             double local_angle_error_d = 0.0;
             double position_error_x_d = 0.0;
             if (sample_time > 0.0) {
                 position_error_x_d = (local_error.x() - previous_position_error.X.error) / sample_time;
                 //position_error_y_d = (local_error.y() - previous_position_error.Y.error) / sample_time;
                 local_angle_error_d = (local_angle_error - previous_angle_error.Z.error) / sample_time;
             }

            output_velocity.linear.x = pd_linear_precision_gains.kp * local_error.x() + pd_linear_precision_gains.kd * position_error_x_d;
            output_velocity.linear.y = pd_linear_precision_gains.kp * local_error.y() + pd_linear_precision_gains.kd * position_error_x_d;
            output_velocity.angular.z = pd_angular_gains.kp * local_angle_error + pd_angular_gains.kd * local_angle_error_d;

            previous_position_error.X.error = local_error.x();
            previous_angle_error.Z.error = local_angle_error;

            output_velocity.linear.x = std::clamp(output_velocity.linear.x, -1.1, 1.1); //Speed limit after Ridgeback Max Speed
    return output_velocity;
}