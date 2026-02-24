cmake_minimum_required(VERSION 3.16)
project(lab_base)

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
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

# Find OpenGL package
find_package(OpenGL REQUIRED)

include_directories(

${CMAKE_SOURCE_DIR}/include
)

# Add executable for the manipulator node
add_executable(lab_chassis
    src/main.cpp
)

target_link_libraries(lab_chassis
  OpenGL::GL 
)

# Ensure ROS 2 dependencies are added correctly
ament_target_dependencies(lab_chassis
  rclcpp
  std_msgs
  interfaces
)

# Install the executable
install(TARGETS lab_chassis
  DESTINATION lib/${PROJECT_NAME}
)


# Ament package
ament_package()
