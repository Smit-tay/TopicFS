# Copyright (c) 2025 Jack Sidman Smith
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

cmake_minimum_required(VERSION 3.8)
project(topic_fs)

# Set C++ standard to 20 (compatible with ROS 2 Jazzy and nlohmann_json)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compiler warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FUSE3 REQUIRED fuse3) # Use 'fuse3' if using FUSE 3.x

# Find nlohmann_json (header-only library)
find_package(nlohmann_json REQUIRED)
if(NOT nlohmann_json_FOUND)
  message(WARNING "nlohmann_json not found via CMake, assuming system package")
endif()

# Add executable
add_executable(topic_fs
  src/main.cpp
  src/topicfs_node.cpp
  src/topicfs_fuse.cpp
  src/topicfs.cpp
)

# Include directories for FUSE and nlohmann_json
target_include_directories(topic_fs PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${FUSE3_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
  #/usr/include/nlohmann # Adjust if nlohmann_json is installed elsewhere
)

# Link libraries for FUSE
target_link_libraries(topic_fs
  ${FUSE3_LIBRARIES}
)

# Specify FUSE compiler flags
target_compile_options(topic_fs PRIVATE ${FUSE_CFLAGS_OTHER})

target_compile_definitions(topic_fs PRIVATE FUSE_USE_VERSION=31)

install(TARGETS
  topic_fs
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)


# ROS 2 dependencies (handles includes and linking)
ament_target_dependencies(topic_fs
#  geometry_msgs
  rclcpp  
#  rosbag2_cpp
  rosidl_runtime_cpp  
#  std_msgs
)

# Install executable
install(TARGETS topic_fs
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, configs, etc. (uncomment if needed)
# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}
# )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(AMENT_LINT_AUTO_CPPLINT_FILTERS "--filter=-whitespace/newline,-readability/braces")
  ament_uncrustify(
    TESTNAME uncrustify
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/uncrustify.cfg
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
