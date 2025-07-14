cmake_minimum_required(VERSION 3.8)
project(aruco_navigator)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/aruco_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

# CMakeLists.txt should include:
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
