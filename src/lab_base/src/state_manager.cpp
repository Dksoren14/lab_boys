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



