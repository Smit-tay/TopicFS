// Copyright 2025 Jack Sidman Smith
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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

private:
  // Load type support for a given type string via dlopen.
  // Results are cached to avoid repeated dlopen calls.
  static const rosidl_message_type_support_t * get_type_support(
    const std::string & type_string);

  // Parse type string into package, subfolder, type components.
  // e.g. "geometry_msgs/msg/Point" -> {"geometry_msgs", "msg", "Point"}
  static bool parse_type_string(
    const std::string & type_string,
    std::string & package,
    std::string & subfolder,
    std::string & type_name);

  // Recursively deserialize raw memory to JSON using member introspection.
  static nlohmann::json members_to_json(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members,
    const uint8_t * data);

  // Recursively serialize JSON to raw memory using member introspection.
  static bool json_to_members(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members,
    const nlohmann::json & json,
    uint8_t * data);

  // Cache of loaded type supports keyed by type string
  static std::unordered_map<std::string, const rosidl_message_type_support_t *> type_support_cache_;
};

#endif  // TOPIC_FS__ROS_MESSAGE_CONVERTER_HPP_
