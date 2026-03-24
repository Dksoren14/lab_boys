#include "transformation.h"



Eigen::Vector3d Transformation::global_to_local(
    const Stamped3DVector& target_vector, 
    const Stamped3DVector& current_position, 
    const Eigen::Vector3d& current_orientation){
    // Create transformation matrix from current orientation (assuming current_orientation is in Euler angles)
    double yaw = current_orientation.x(); // Roll is for now the rotation around the z-axis (change later if needed) (in radians)
    Eigen::Matrix3d transformation_matrix;
    transformation_matrix << cos(yaw), -sin(yaw), current_position.x(),
                             sin(yaw),  cos(yaw), current_position.y(),
                                  0,       0,     1;
    // Convert target vector to homogeneous coordinates
    Eigen::Vector3d homogeneous_target_vector = Eigen::Vector3d(target_vector.x(), target_vector.y(), 1); //Creates target vector for local reference frame origin position
    // Apply inverse transformation to get local coordinates
    Eigen::Vector3d local_vector = transformation_matrix.inverse() * homogeneous_target_vector;
    double hyp = sqrt(local_vector.x() * local_vector.x() + local_vector.y() * local_vector.y());
    double local_yaw = asin(local_vector.y() / hyp);
    local_vector.z() = local_yaw;
    return local_vector;
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
    yaw = unwrapAngle(yaw, 2 * M_PI, 0);
    pitch = unwrapAngle(pitch, 2 * M_PI, 0);
    roll = unwrapAngle(roll, 2 * M_PI, 0);

    // Ensure yaw is in [0, 2π] and consistent with input
    return Eigen::Vector3d(yaw, pitch, roll);
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

