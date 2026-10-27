cmake_minimum_required(VERSION 3.8)
project(robotic_arm_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(arm_controller src/arm_controller.cpp)
ament_target_dependencies(arm_controller rclcpp rclcpp_action std_msgs robot_msgs)
target_link_libraries(arm_controller Eigen3::Eigen)

add_executable(gripper_controller src/gripper_controller.cpp)
ament_target_dependencies(gripper_controller rclcpp std_msgs std_srvs ros_gz_interfaces sensor_msgs)

add_executable(main_controller src/main_controller.cpp)
ament_target_dependencies(main_controller rclcpp rclcpp_action std_srvs robot_msgs)

install(TARGETS arm_controller gripper_controller main_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config world models
  DESTINATION share/${PROJECT_NAME}
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
