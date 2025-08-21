cmake_minimum_required(VERSION 3.8)
project(spina_arm_controll)
#set(PROJECT_MAIN spina_arm_controll)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(serial_controller_node src/seSe_arm_controll.cpp)
ament_target_dependencies(serial_controller_node rclcpp std_msgs)

add_executable(angle_send_node src/angle_send.cpp)
ament_target_dependencies(angle_send_node rclcpp std_msgs)

add_executable(angle_cmd_publisher_node src/angle_cmd_publisher.cpp)
ament_target_dependencies(angle_cmd_publisher_node rclcpp std_msgs)

add_executable(joy_spina_node src/joy_spina.cpp)
ament_target_dependencies(joy_spina_node rclcpp sensor_msgs)

install(TARGETS
  serial_controller_node
  angle_send_node
  angle_cmd_publisher_node
  joy_spina_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
