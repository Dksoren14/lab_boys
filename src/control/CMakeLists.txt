cmake_minimum_required(VERSION 3.5)
project(control) #changed from "lab_base"

# Use C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# Fix OpenGL GLVND issue by setting the policy
if (POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)


# Find OpenGL package
find_package(OpenGL REQUIRED)

include_directories(include)

# Add executable for the manipulator node
add_executable(lab_chassis
    src/main.cpp
    src/state_manager.cpp
    src/controller.cpp
    src/transformation.cpp
)
target_include_directories(lab_chassis PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lab_chassis 
  OpenGL::GL 
)


# Ensure ROS 2 dependencies are added correctly
ament_target_dependencies(lab_chassis
  rclcpp
  rclcpp_action
  std_msgs
  interfaces
  nav2_msgs
)

# Install the executable
install(TARGETS lab_chassis
  DESTINATION lib/${PROJECT_NAME}
)
# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)



# Ament package
ament_package()
