cmake_minimum_required(VERSION 3.8)
project(panda_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(PROGRAMS
  scripts/arm_control_from_UI.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
