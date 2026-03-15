#include "state_manager.h"

void StateManager::setLocalPosition(const Stamped3DVector& position) {
    std::lock_guard<std::mutex> lock(local_position_mutex);
    local_position = position;
}

Stamped3DVector StateManager::getLocalPosition() {
    std::lock_guard<std::mutex> lock(local_position_mutex);
    return local_position;
}



