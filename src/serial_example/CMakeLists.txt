cmake_minimum_required(VERSION 3.5)
project(serial_example)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-Wall -Werror)

## C++ 14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)


add_library(${PROJECT_NAME}
  src/serial_example.cpp
)

ament_target_dependencies(serial_example
    rclcpp 
    serial_driver
    std_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN SerialPortNode
  EXECUTABLE ${PROJECT_NAME}_node
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


# Install executable
install(TARGETS serial_example_node DESTINATION lib/${PROJECT_NAME}_node)

# 添加生成launch文件所需的依赖
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}_node
)

# Export dependencies
ament_export_dependencies(rclcpp serial_driver)
 
ament_package()

