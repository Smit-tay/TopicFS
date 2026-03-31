// ros_message_converter.hpp
// Copyright 2025 Jack Sidman Smith
//
// Licensed under the MIT License. See LICENSE in project root.

#ifndef TOPIC_FS__ROS_MESSAGE_CONVERTER_HPP_
#define TOPIC_FS__ROS_MESSAGE_CONVERTER_HPP_

// Standard library
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

// Third-party
#include <nlohmann/json.hpp>
#include <rosx_introspection/ros_parser.hpp>

// ROS2
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

class RosMessageConverter
{
public:
  // Register a rosx_introspection Parser for a type, built from the
  // concatenated message definition obtained via GetTypeDescription.
  // type_string format: "geometry_msgs/msg/Point"
  // definition: concatenated ROS1-style definition (primary + dependencies
  //             separated by "================..." / "MSG: pkg/Type" lines).
  // Safe to call from any thread. No-op if already registered.
  static void register_definition(
    const std::string & type_string,
    const std::string & definition);

  // Convert a serialized ROS2 message to human-readable JSON.
  // Resolution order:
  //   1. rosx_introspection Parser (zero-config, populated via GetTypeDescription)
  //   2. dlopen typesupport .so   (legacy, requires AMENT_PREFIX_PATH)
  //   3. std::nullopt             (caller falls back to base64 CDR)
  // type_string format: "geometry_msgs/msg/Point"
  static std::optional<nlohmann::json> to_json(
    const std::string & type_string,
    const rclcpp::SerializedMessage & msg);

  // Convert human-readable JSON to a serialized ROS2 message ready to publish.
  // Uses dlopen path only — rosx_introspection is deserialize-only.
  // type_string format: "geometry_msgs/msg/Point"
  static std::optional<rclcpp::SerializedMessage> from_json(
    const std::string & type_string,
    const nlohmann::json & json);

  // Load type support for a given type string via dlopen.
  // Results are cached. Used by from_json() and call_service() in topicfs_node.
  static const rosidl_message_type_support_t * get_type_support(
    const std::string & type_string);

  // Recursively deserialize raw message memory to JSON using member introspection.
  // Used directly by call_service() for service response deserialization.
  static nlohmann::json members_to_json(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members,
    const uint8_t * data);

  // Recursively serialize JSON to raw message memory using member introspection.
  // Used directly by call_service() for service request serialization.
  static bool json_to_members(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members,
    const nlohmann::json & json,
    uint8_t * data);

private:
  // Parse "geometry_msgs/msg/Point" -> package="geometry_msgs",
  //   subfolder="msg", type_name="Point".
  static bool parse_type_string(
    const std::string & type_string,
    std::string & package,
    std::string & subfolder,
    std::string & type_name);

  // Convert "geometry_msgs/msg/Point" -> "geometry_msgs/Point"
  // (the format rosx_introspection ROSType expects).
  static std::string to_rosx_type(const std::string & type_string);

  // ── rosx_introspection parser cache ────────────────────────────────────────
  // Keyed by type_string ("geometry_msgs/msg/Point").
  // Populated by register_definition(), consumed by to_json().
  // Parser is not copyable — stored by unique_ptr.
  static std::mutex                                                 parser_mutex_;
  static std::unordered_map<std::string,
    std::unique_ptr<RosMsgParser::Parser>>                          parser_cache_;

  // Tracks types for which GetTypeDescription has already been requested,
  // so we don't fire duplicate async calls during repeated discovery ticks.
  static std::mutex                                                 requested_mutex_;
  static std::unordered_set<std::string>                            requested_types_;

  // ── dlopen typesupport cache ───────────────────────────────────────────────
  static std::unordered_map<std::string,
    const rosidl_message_type_support_t *>                          type_support_cache_;

public:
  // Returns true if a GetTypeDescription request has already been fired for
  // this type_string. Used by topicfs_node to avoid duplicate requests.
  static bool mark_requested(const std::string & type_string);
};

#endif  // TOPIC_FS__ROS_MESSAGE_CONVERTER_HPP_
