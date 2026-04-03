cmake_minimum_required(VERSION 3.8)
project(pivot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CURL_INCLUDE_DIRS})

# Library: PIVOT Planner Core
add_library(pivot_planner_core
  src/types.cpp
  src/feature_extractor.cpp
  src/sampler.cpp
  src/pivot_sampler.cpp
  src/speculation.cpp
  src/pivot_rrt_star.cpp
)

ament_target_dependencies(pivot_planner_core
  rclcpp
  geometry_msgs
  nav_msgs
)

# Simple Map Publisher
add_executable(simple_map_publisher src/simple_map_publisher.cpp)
ament_target_dependencies(simple_map_publisher rclcpp nav_msgs)
target_link_libraries(simple_map_publisher ${OpenCV_LIBS})

# Multi Path Planner
add_executable(multi_path_planner src/multi_path_planner.cpp)
target_link_libraries(multi_path_planner pivot_planner_core)
ament_target_dependencies(multi_path_planner rclcpp geometry_msgs nav_msgs)

# Path Follower
add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower
  rclcpp
  nav_msgs
  geometry_msgs
)

# Prompt-based Planner with OpenAI
add_executable(prompt_pivot_planner src/prompt_pivot_planner.cpp)
target_link_libraries(prompt_pivot_planner 
  pivot_planner_core
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(prompt_pivot_planner
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# Install executables
install(TARGETS
  pivot_planner_core
  simple_map_publisher
  multi_path_planner
  path_follower
  prompt_pivot_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY
  maps
  launch
  config
  gazebo_worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()