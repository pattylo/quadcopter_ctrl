cmake_minimum_required(VERSION 3.8)
project(quadcopter_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    include
    ${Sophus_INCLUDE_DIRS}
)

###################################################################
# ctrl_server
add_executable(ctrl_server_node
	src/ctrl_server_node.cpp
    src/ctrl_server.cpp
)
ament_target_dependencies(ctrl_server_node
	rclcpp 
	geometry_msgs
	sensor_msgs
	mavros_msgs
	visualization_msgs
    # bezier_traj_msgs
)
target_link_libraries(ctrl_server_node
    Sophus::Sophus
)
install(TARGETS ctrl_server_node DESTINATION lib/${PROJECT_NAME})

###################################################################
# Install Python scripts
install(PROGRAMS
  script/offboard.py
  DESTINATION lib/${PROJECT_NAME}
)

###################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
