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
#find_package(YARP REQUIRED)
#find_package(OpenCV REQUIRED)

#project(${PROJECT_MAIN})

add_executable(serial_controller_node src/seSe_arm_controll.cpp)
ament_target_dependencies(serial_controller_node rclcpp std_msgs)

add_executable(angle_send_node src/angle_send.cpp)
ament_target_dependencies(angle_send_node rclcpp std_msgs)

add_executable(angle_cmd_publisher_node src/angle_cmd_publisher.cpp)
ament_target_dependencies(angle_cmd_publisher_node rclcpp std_msgs)

#target_link_libraries(${PROJECT_MAIN}
#	${YARP_LIBRARIES}
#	${OpenCV_LIBS}
	#${ROBOT_LIBRARY}
#)

install(TARGETS
  serial_controller_node
  angle_send_node
  angle_cmd_publisher_node
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
