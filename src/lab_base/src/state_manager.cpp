#include "state_manager.h"

// Non used position - obtains error
void StateManager::setLocalPosition(const Stamped3DVector& position) {
    std::lock_guard<std::mutex> lock(local_position_mutex);
    local_position = position;
}

Stamped3DVector StateManager::getLocalPosition() {
    std::lock_guard<std::mutex> lock(local_position_mutex);
    return local_position;
}

void StateManager::setGlobalBasePosition(const Stamped3DVector& position) {
    std::lock_guard<std::mutex> lock(global_base_position_mutex);
    global_base_position = position;
}

Stamped3DVector StateManager::getGlobalBasePosition() {
    std::lock_guard<std::mutex> lock(global_base_position_mutex);
    return global_base_position;
}
void StateManager::setGlobalBaseOrientation(const Eigen::Quaterniond& orientation) {
    std::lock_guard<std::mutex> lock(global_base_orientation_mutex);
    global_base_orientation = orientation;
}

Eigen::Quaterniond StateManager::getGlobalBaseOrientation() {
    std::lock_guard<std::mutex> lock(global_base_orientation_mutex);
    return global_base_orientation;
}

void StateManager::setLocalVelocity(const Stamped3DVector& velocity) {
    std::lock_guard<std::mutex> lock(local_velocity_mutex);
    local_velocity = velocity;
}

Stamped3DVector StateManager::getLocalVelocity() {
    std::lock_guard<std::mutex> lock(local_velocity_mutex);
    return local_velocity;
}

void StateManager::setTargetPosition(const Stamped3DVector& target_position_) {
    std::lock_guard<std::mutex> lock(target_position_mutex);
    target_position = target_position_;
}


Stamped3DVector StateManager::getTargetPosition() {
    std::lock_guard<std::mutex> lock(target_position_mutex);
    return target_position;
}

void StateManager::setControlMode(const int& mode) {
    std::lock_guard<std::mutex> lock(control_mode_mutex);
    control_mode = mode;
}

int StateManager::getControlMode() {
    std::lock_guard<std::mutex> lock(control_mode_mutex);
    return control_mode;
}

void StateManager::setPositionError(const PositionError& error) {
    std::lock_guard<std::mutex> lock(position_error_mutex);
    position_error = error;
}

PositionError StateManager::getPositionError() {
    std::lock_guard<std::mutex> lock(position_error_mutex);
    return position_error;
}

void StateManager::setGoalPosition(const Stamped3DVector& goal_position_) {
    std::lock_guard<std::mutex> lock(goal_position_mutex);
    goal_position = goal_position_;
}

Stamped3DVector StateManager::getGoalPosition() {
    std::lock_guard<std::mutex> lock(goal_position_mutex);
    return goal_position;
}

