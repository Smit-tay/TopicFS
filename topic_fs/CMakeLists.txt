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

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rmw REQUIRED)
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

# -----------------------------------------------------------------------------
# Executable
# -----------------------------------------------------------------------------

# Add executable
add_executable(topic_fs
  src/main.cpp
  src/topicfs_node.cpp
  src/topicfs_fuse.cpp
  src/topicfs.cpp
  src/ros_message_converter.cpp
)


# We require 3.12 (or higher ?)
target_compile_definitions(topic_fs PRIVATE FUSE_USE_VERSION=312)

# Include directories for FUSE and nlohmann_json
target_include_directories(topic_fs PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${FUSE3_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

# Link libraries for FUSE
target_link_libraries(topic_fs
  ${FUSE3_LIBRARIES}
  nlohmann_json::nlohmann_json
  dl
)

# Specify FUSE compiler flags
target_compile_options(topic_fs PRIVATE ${FUSE_CFLAGS_OTHER})

# All ROS2 includes and linking — note: no message packages here.
# TopicFS resolves all message types dynamically at runtime via dlopen/dlsym.
# Third-party message .so files are found automatically provided their
# containing install prefix is on AMENT_PREFIX_PATH (i.e. setup.bash sourced).
ament_target_dependencies(topic_fs
  rclcpp
  rclcpp_action
  rcl_action
  rmw
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------

install(TARGETS
  topic_fs
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)




# Install executable
install(TARGETS topic_fs
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
