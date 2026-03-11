cmake_minimum_required(VERSION 3.16)
project(lab_manipulator)

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
find_package(rclcpp_action REQUIRED)

# Find OpenGL package
find_package(OpenGL REQUIRED)

include_directories(

${CMAKE_SOURCE_DIR}/include
)

# Add executable for the manipulator node
add_executable(lab_arm
    src/main.cpp
)

target_link_libraries(lab_arm
  OpenGL::GL 
)

# Ensure ROS 2 dependencies are added correctly
ament_target_dependencies(lab_arm
  rclcpp
  rclcpp_action
  std_msgs
  interfaces
)

# Install the executable
install(TARGETS lab_arm
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install world
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)



# Ament package
ament_package()
