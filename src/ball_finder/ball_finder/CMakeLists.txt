cmake_minimum_required(VERSION 3.8)
project(ball_finder)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallLocation.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

install(PROGRAMS
  ball_finder/BallFinder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()