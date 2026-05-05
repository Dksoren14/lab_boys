#include "transformation.h"



Eigen::Vector3d Transformation::global_to_local(
    const Stamped3DVector& target_vector, 
    const Stamped3DVector& current_position, 
    const Eigen::Vector3d& current_orientation){
    // Create transformation matrix from current orientation (assuming current_orientation is in Euler angles)
    double yaw = current_orientation.x();
    Eigen::Matrix3d transformation_matrix;
    transformation_matrix << cos(yaw), -sin(yaw), current_position.x(),
                             sin(yaw),  cos(yaw), current_position.y(),
                                  0,       0,     1;
    // Convert target vector to homogeneous coordinates
    Eigen::Vector3d homogeneous_target_vector = Eigen::Vector3d(target_vector.x(), target_vector.y(), 1); //Creates target vector for local reference frame origin position
    // Apply inverse transformation to get local coordinates
    Eigen::Vector3d local_vector = transformation_matrix.inverse() * homogeneous_target_vector;
  
    return local_vector;
}   

Eigen::Vector3d Transformation::global_to_local_error(
    const Stamped3DVector& current_position,
    Eigen::Vector3d& global_error,
    const Eigen::Vector3d& current_orientation
){

    double yaw = current_orientation.z();

    double local_x =  cos(yaw) * global_error.x() + sin(yaw) * global_error.y();
    double local_y = -sin(yaw) * global_error.x() + cos(yaw) * global_error.y();

    return Eigen::Vector3d(local_x, local_y, 0.0);
}



double Transformation::calculate_angle_to_target(
    const Stamped3DVector& target_vector,
    const Stamped3DVector& current_position, 
    const Eigen::Vector3d& current_orientation) {


    double yaw = current_orientation.z();

    //Eigen::Matrix3d transformation_matrix;
    //transformation_matrix << cos(yaw), -sin(yaw), 0,
    //                         sin(yaw),  cos(yaw), 0,
    //                              0,       0,     1;   
    //Eigen::Vector3d homogeneous_target_vector = Eigen::Vector3d(target_vector.x(), target_vector.y(), 1);
    //Eigen::Vector3d local_vector = transformation_matrix.inverse() * homogeneous_target_vector;
//
    //double local_angle_error = atan2(local_vector.y(), local_vector.x());
    //return local_angle_error; The old way, aparently wrong

    double dx = target_vector.x() - current_position.x();
    double dy = target_vector.y() - current_position.y();
    
    double local_x = cos(yaw) * dx + sin(yaw) * dy;
    double local_y = -sin(yaw) * dx + cos(yaw) * dy;

    return atan2(local_y, local_x);
}







Eigen::Vector3d Transformation::quaternion_to_euler(const Eigen::Quaterniond& q) const {
    // Get Euler angles in ZYX convention (yaw, pitch, roll)
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    
    // Extract yaw, pitch, roll
    double yaw = euler.x();
    double pitch = euler.y();
    double roll = euler.z();

    // Normalize pitch to [-π/2, π/2] to avoid gimbal lock ambiguities
    if (std::abs(pitch) > M_PI / 2) {
        // Adjust yaw and flip pitch and roll to maintain equivalent rotation
        yaw += M_PI;
        pitch = M_PI - pitch; // Reflect pitch around π
        roll += M_PI;
    }

    // Unwrap angles to [0, 2π]
    yaw = unwrapAngle(yaw, M_PI, -M_PI);
    pitch = unwrapAngle(pitch, 2 * M_PI, 0);
    roll = unwrapAngle(roll, 2 * M_PI, 0);

    // Ensure yaw is in [0, 2π] and consistent with input
    return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond Transformation::euler_to_quaternion(double roll, double pitch, double yaw) const {
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q.normalized();
}

double Transformation::unwrapAngle(double angle, double max, double min) const {
    // Unwrap the angle to be within the range [min, max]    
    while (angle > max) angle -= 2.0 * M_PI;
    while (angle < min) angle += 2.0 * M_PI;
    return angle;
    }

double Transformation::velocity_vector(const Stamped3DVector& current_position, Stamped3DVector& previous_position, double d_time){
    
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    if(d_time > 0.0){
        velocity_x = (current_position.x() - previous_position.x()) / d_time;
        velocity_y = (current_position.y() - previous_position.y()) / d_time;
    }
   
    previous_position.setX(current_position.x());
    previous_position.setY(current_position.y());

    return std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
}

Stamped3DVector Transformation::aruco_translation(
    const Stamped3DVector& aruco_vector,
    const Eigen::Vector3d& aruco_orientation,
    const Stamped3DVector& current_position,
    const Eigen::Vector3d& current_orientation)
{
    // Transform 1: ArUco marker pose in global frame
    // Rotation from aruco's yaw, translation from aruco's global position
    Eigen::Matrix4d T_G_A = Eigen::Matrix4d::Identity();
    double yaw = aruco_orientation.z();
    T_G_A(0,0) =  cos(yaw);  T_G_A(0,1) = -sin(yaw);
    T_G_A(1,0) =  sin(yaw);  T_G_A(1,1) =  cos(yaw);
    T_G_A(0,3) = aruco_vector.x();
    T_G_A(1,3) = aruco_vector.y();
    T_G_A(2,3) = aruco_vector.z();

    // Transform 2: Offset point in ArUco's LOCAL frame
    // -0.875 along aruco's local X = "behind" the marker
    Eigen::Matrix4d T_A_F = Eigen::Matrix4d::Identity();
    T_A_F(0,3) = -0.875;

    // Compose: global pose of the offset point
    Eigen::Matrix4d T_G_F = T_G_A * T_A_F;

    return Stamped3DVector(aruco_vector.getTime(), T_G_F(0,3), T_G_F(1,3), T_G_F(2,3));
}
