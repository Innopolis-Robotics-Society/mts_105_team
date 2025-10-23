cmake_minimum_required(VERSION 3.10)
project(a105_cobra_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cmdvel_to_cobra src/cmdvel_to_cobra.cpp)
ament_target_dependencies(cmdvel_to_cobra rclcpp geometry_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS cmdvel_to_cobra RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
