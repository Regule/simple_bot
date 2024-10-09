cmake_minimum_required(VERSION 3.8)
project(simple_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  example_interfaces
)

add_executable(tiny_node src/tiny_node.cpp)
add_executable(small_node_pub src/small_node_pub.cpp)
add_executable(small_node_sub src/small_node_sub.cpp)
add_executable(parametrized_publisher src/parametrized_publisher.cpp)
add_executable(fake_lidar src/fake_lidar.cpp)

ament_target_dependencies(tiny_node ${dependencies})
ament_target_dependencies(small_node_pub ${dependencies})
ament_target_dependencies(small_node_sub ${dependencies})
ament_target_dependencies(parametrized_publisher ${dependencies})
ament_target_dependencies(fake_lidar ${dependencies})

install(TARGETS
  tiny_node
  small_node_pub
  small_node_sub
  parametrized_publisher
  fake_lidar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch DESTINATION share/${PROJECT_NAME}
)
    
ament_python_install_package(${PROJECT_NAME})
# Following part must include all scripts with nodes.
install(PROGRAMS
    scripts/pypublisher.py
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

ament_export_dependencies(${dependencies})
ament_package()
