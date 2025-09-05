cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_teleop)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlebot3_manipulation_bringup REQUIRED)
find_package(turtlebot3_manipulation_moveit_config REQUIRED)
find_package(turtlebot3_manipulation_description REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(turtlebot3_manipulation_teleop src/turtlebot3_manipulation_teleop.cpp)
target_include_directories(turtlebot3_manipulation_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  turtlebot3_manipulation_teleop
  "rclcpp"
  "turtlebot3_manipulation_bringup"
  "turtlebot3_manipulation_moveit_config"
  "turtlebot3_manipulation_description"
  "geometry_msgs"
  "nav_msgs"
  "control_msgs"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_action"
)

add_executable(pick_moveit src/pick_moveit.cpp)
target_include_directories(pick_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pick_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)

install(TARGETS 
	turtlebot3_manipulation_teleop
	pick_moveit
  	DESTINATION lib/${PROJECT_NAME}
  	)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
