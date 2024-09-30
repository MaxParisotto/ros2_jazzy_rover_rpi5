cmake_minimum_required(VERSION 3.8)
project(my_custom_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

# Add C++ executables
add_executable(encoder_subscriber src/encoder_subscriber.cpp)
ament_target_dependencies(encoder_subscriber rclcpp std_msgs)

add_executable(motor_command_publisher src/motor_command_publisher.cpp)
ament_target_dependencies(motor_command_publisher rclcpp std_msgs)

add_executable(imu_subscriber src/imu_subscriber.cpp)
ament_target_dependencies(imu_subscriber rclcpp sensor_msgs)

# Install the launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install executables
install(TARGETS
  encoder_subscriber
  motor_command_publisher
  imu_subscriber
  DESTINATION lib/${PROJECT_NAME})

# Install Python script
install(PROGRAMS
  scripts/save_map_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()