cmake_minimum_required(VERSION 3.8)
project(racecar_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings (optional)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build the executable
add_executable(racecar_controller
  src/racecar_controller.cpp
)

# Link dependencies to the executable
ament_target_dependencies(racecar_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  racecar_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Linting and testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize ROS 2 package setup
ament_package()