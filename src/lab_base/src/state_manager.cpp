#include "state_manager.h"

void StateManager::setLocalPosition(const Stamped3DVector& position) {
    std::lock_guard<std::mutex> lock(local_position_mutex);
    local_position = position;
}

Stamped3DVector StateManager::getLocalPosition() {
    std::lock_guard<std::mutex> lock(local_position_mutex);
    return local_position;
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