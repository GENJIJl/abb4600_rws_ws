cmake_minimum_required(VERSION 3.10.2)
project(abb_irb4600_60_205_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)
