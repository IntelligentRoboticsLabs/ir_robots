cmake_minimum_required(VERSION 3.8)
project(ir_robots)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config params maps worlds
        DESTINATION share/${PROJECT_NAME})

ament_package()
