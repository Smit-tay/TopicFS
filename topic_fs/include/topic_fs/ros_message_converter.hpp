// ros_message_converter.hpp
// Copyright 2025 Jack Sidman Smith
//
// Licensed under the MIT License. See LICENSE in project root.

#ifndef TOPIC_FS__ROS_MESSAGE_CONVERTER_HPP_
#define TOPIC_FS__ROS_MESSAGE_CONVERTER_HPP_

// Standard library
#include <optional>
#include <string>
#include <unordered_map>

// Third-party
#include <nlohmann/json.hpp>

// ROS2
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

class RosMessageConverter
{
public:
  // Convert a serialized ROS2 message to human-readable JSON.
  // type_string format: "geometry_msgs/msg/Point"
  // Returns empty optional on failure.
  static std::optional<nlohmann::json> to_json(
    const std::string & type_string,
    const rclcpp::SerializedMessage & msg);

  // Convert human-readable JSON to a serialized ROS2 message ready to publish.
  // type_string format: "geometry_msgs/msg/Point"
  // Returns empty optional on failure.
  static std::optional<rclcpp::SerializedMessage> from_json(
    const std::string & type_string,
    const nlohmann::json & json);

  // Load type support for a given type string via dlopen.
  // The .so is found automatically provided the package's install prefix
  // is on AMENT_PREFIX_PATH (i.e. its setup.bash has been sourced), which
  // causes LD_LIBRARY_PATH to be set correctly. No manual path management
  // is needed or performed here. Results are cached.
  static const rosidl_message_type_support_t * get_type_support(
    const std::string & type_string);

  // Recursively deserialize raw memory to JSON using member introspection.
  static nlohmann::json members_to_json(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members,
    const uint8_t * data);

  // Recursively serialize JSON to raw memory using member introspection.
  static bool json_to_members(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members,
    const nlohmann::json & json,
    uint8_t * data);

private:
  // Parse type string into package, subfolder, type components.
  // e.g. "geometry_msgs/msg/Point" -> {"geometry_msgs", "msg", "Point"}
  static bool parse_type_string(
    const std::string & type_string,
    std::string & package,
    std::string & subfolder,
    std::string & type_name);

  // Cache of loaded type supports keyed by type string.
  static std::unordered_map<std::string, const rosidl_message_type_support_t *>
    type_support_cache_;
};

#endif  // TOPIC_FS__ROS_MESSAGE_CONVERTER_HPP_
