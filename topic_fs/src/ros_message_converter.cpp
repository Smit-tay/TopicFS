// ros_message_converter.cpp
// Copyright 2025 Jack Sidman Smith
//
// Licensed under the MIT License. See LICENSE in project root.

// Standard library
#include <cstring>
#include <dlfcn.h>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Third-party
#include <nlohmann/json.hpp>
#include <rosx_introspection/ros_parser.hpp>
#include <rosx_introspection/deserializer.hpp>

// ROS2
#include <rclcpp/serialized_message.hpp>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

// there seems to be some confusion about which span is used.
// #define span_CONFIG_SELECT_SPAN=1
#include <rosx_introspection/contrib/span.hpp>

// Project
#include "topic_fs/ros_message_converter.hpp"

// -----------------------------------------------------------------------------
// Static member definitions
// -----------------------------------------------------------------------------

std::mutex RosMessageConverter::parser_mutex_;
std::unordered_map<std::string,
  std::unique_ptr<RosMsgParser::Parser>> RosMessageConverter::parser_cache_;

std::mutex RosMessageConverter::requested_mutex_;
std::unordered_set<std::string> RosMessageConverter::requested_types_;

std::unordered_map<std::string,
  const rosidl_message_type_support_t *> RosMessageConverter::type_support_cache_;

// -----------------------------------------------------------------------------
// Public - register_definition
//
// Called from topicfs_node after a successful GetTypeDescription response.
// Constructs a rosx_introspection Parser and caches it for use by to_json().
//
// definition is the concatenated ROS1-style message definition: the primary
// type's .msg content followed by zero or more dependency blocks separated by
// the standard "=====..." / "MSG: pkg/Type" header lines, assembled by
// topicfs_node from the type_sources array in the GetTypeDescription response.
// -----------------------------------------------------------------------------

void RosMessageConverter::register_definition(
  const std::string & type_string,
  const std::string & definition)
{
  std::lock_guard<std::mutex> lock(parser_mutex_);
  if (parser_cache_.count(type_string))
  {
    return;  // Already registered
  }

  try
  {
    // rosx_introspection expects "geometry_msgs/Point", not "geometry_msgs/msg/Point"
    const std::string rosx_type = to_rosx_type(type_string);
    auto parser = std::make_unique<RosMsgParser::Parser>(
      type_string,                    // topic_name — used as tree root label
      RosMsgParser::ROSType(rosx_type),
      definition);

    parser_cache_[type_string] = std::move(parser);
    fprintf(stderr,
      "[RosMessageConverter] registered rosx parser for '%s'\n",
      type_string.c_str());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr,
      "[RosMessageConverter] register_definition failed for '%s': %s\n",
      type_string.c_str(), e.what());
  }
}

// -----------------------------------------------------------------------------
// Public - mark_requested
//
// Atomically marks a type as having had GetTypeDescription requested.
// Returns true if this call is the first (caller should fire the request).
// Returns false if already requested (caller should skip — duplicate).
// -----------------------------------------------------------------------------

bool RosMessageConverter::mark_requested(const std::string & type_string)
{
  std::lock_guard<std::mutex> lock(requested_mutex_);
  auto [_, inserted] = requested_types_.insert(type_string);
  return inserted;
}

// -----------------------------------------------------------------------------
// Public - to_json
//
// Resolution order:
//   1. rosx_introspection Parser — populated asynchronously via GetTypeDescription.
//      Zero config: works for any type the remote node knows about.
//   2. dlopen typesupport .so — legacy path, requires AMENT_PREFIX_PATH.
//   3. std::nullopt — caller (subscribe_to_topic) falls back to base64 CDR.
//
// ROS2 CDR buffers have a 4-byte representation identifier header prepended
// by the RMW layer. rosx_introspection operates on raw CDR payload, so we
// skip those 4 bytes before passing the buffer.
// -----------------------------------------------------------------------------

std::optional<nlohmann::json> RosMessageConverter::to_json(
  const std::string & type_string,
  const rclcpp::SerializedMessage & msg)
{
  const auto & rcl_msg = msg.get_rcl_serialized_message();
  if (!rcl_msg.buffer || rcl_msg.buffer_length == 0)
  {
    return std::nullopt;
  }

  // ── Tier 1: rosx_introspection ─────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lock(parser_mutex_);
    auto it = parser_cache_.find(type_string);
    if (it != parser_cache_.end())
    {
      // Strip the 4-byte ROS2 CDR representation header
      constexpr size_t cdr_header = 4;
      if (rcl_msg.buffer_length <= cdr_header)
      {
        return std::nullopt;
      }

      nonstd::span<const uint8_t> span(
        rcl_msg.buffer + cdr_header,
        rcl_msg.buffer_length - cdr_header);

      std::string json_txt;
      RosMsgParser::ROS2_Deserializer deserializer;
      deserializer.init(span);

      bool ok = it->second->deserializeIntoJson(span, &json_txt, &deserializer);
      if (ok && !json_txt.empty())
      {
        try
        {
          return nlohmann::json::parse(json_txt);
        }
        catch (const nlohmann::json::parse_error &)
        {
          // rosx produced malformed JSON — fall through to dlopen
        }
      }
    }
  }

  // ── Tier 2: dlopen typesupport .so ─────────────────────────────────────────
  const rosidl_message_type_support_t * type_support = get_type_support(type_string);
  if (!type_support)
  {
    return std::nullopt;
  }

  const rosidl_message_type_support_t * introspection_ts =
    get_message_typesupport_handle(
      type_support,
      "rosidl_typesupport_introspection_cpp");
  if (!introspection_ts)
  {
    return std::nullopt;
  }

  const auto * members =
    reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      introspection_ts->data);
  if (!members)
  {
    return std::nullopt;
  }

  std::vector<uint8_t> buffer(members->size_of_);
  members->init_function(buffer.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  serialized_msg.buffer          = rcl_msg.buffer;
  serialized_msg.buffer_length   = rcl_msg.buffer_length;
  serialized_msg.buffer_capacity = rcl_msg.buffer_capacity;
  serialized_msg.allocator       = rcl_msg.allocator;

  rmw_ret_t ret = rmw_deserialize(&serialized_msg, type_support, buffer.data());
  if (ret != RMW_RET_OK)
  {
    members->fini_function(buffer.data());
    return std::nullopt;
  }

  nlohmann::json result = members_to_json(members, buffer.data());
  members->fini_function(buffer.data());
  return result;
}

// -----------------------------------------------------------------------------
// Public - from_json
//
// dlopen path only — rosx_introspection has no serialization support.
// Unchanged from original implementation.
// -----------------------------------------------------------------------------

std::optional<rclcpp::SerializedMessage> RosMessageConverter::from_json(
  const std::string & type_string,
  const nlohmann::json & json)
{
  const rosidl_message_type_support_t * type_support = get_type_support(type_string);
  if (!type_support)
  {
    return std::nullopt;
  }

  const rosidl_message_type_support_t * introspection_ts =
    get_message_typesupport_handle(
      type_support,
      "rosidl_typesupport_introspection_cpp");
  if (!introspection_ts)
  {
    return std::nullopt;
  }

  const auto * members =
    reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      introspection_ts->data);
  if (!members)
  {
    return std::nullopt;
  }

  std::vector<uint8_t> buffer(members->size_of_);
  members->init_function(buffer.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  if (!json_to_members(members, json, buffer.data()))
  {
    members->fini_function(buffer.data());
    return std::nullopt;
  }

  rclcpp::SerializedMessage serialized_msg;
  rmw_serialized_message_t & rcl_msg = serialized_msg.get_rcl_serialized_message();

  rmw_ret_t ret = rmw_serialize(buffer.data(), type_support, &rcl_msg);
  members->fini_function(buffer.data());

  if (ret != RMW_RET_OK)
  {
    return std::nullopt;
  }

  return serialized_msg;
}

// -----------------------------------------------------------------------------
// Public - get_type_support
// Unchanged from original implementation.
// -----------------------------------------------------------------------------

const rosidl_message_type_support_t * RosMessageConverter::get_type_support(
  const std::string & type_string)
{
  auto it = type_support_cache_.find(type_string);
  if (it != type_support_cache_.end())
  {
    return it->second;
  }

  std::string package, subfolder, type_name;
  if (!parse_type_string(type_string, package, subfolder, type_name))
  {
    fprintf(stderr,
      "[RosMessageConverter] Failed to parse type string: '%s'\n",
      type_string.c_str());
    return nullptr;
  }

  std::string ts_lib = "lib" + package + "__rosidl_typesupport_cpp.so";
  void * ts_handle = dlopen(ts_lib.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!ts_handle)
  {
    fprintf(stderr,
      "[RosMessageConverter] dlopen failed for '%s': %s\n"
      "  Ensure the package's install prefix is on AMENT_PREFIX_PATH.\n",
      ts_lib.c_str(), dlerror());
    return nullptr;
  }

  std::string intr_lib = "lib" + package + "__rosidl_typesupport_introspection_cpp.so";
  void * intr_handle = dlopen(intr_lib.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!intr_handle)
  {
    fprintf(stderr,
      "[RosMessageConverter] Warning: dlopen failed for '%s': %s\n"
      "  Continuing — may be loaded transitively.\n",
      intr_lib.c_str(), dlerror());
  }

  std::string symbol =
    "rosidl_typesupport_cpp__get_message_type_support_handle__" +
    package + "__" + subfolder + "__" + type_name;

  using GetTypeSupportFn = const rosidl_message_type_support_t * (*)();
  auto fn = reinterpret_cast<GetTypeSupportFn>(dlsym(ts_handle, symbol.c_str()));
  if (!fn)
  {
    fprintf(stderr,
      "[RosMessageConverter] dlsym failed for symbol '%s' in '%s': %s\n",
      symbol.c_str(), ts_lib.c_str(), dlerror());
    return nullptr;
  }

  const rosidl_message_type_support_t * ts = fn();
  type_support_cache_[type_string] = ts;
  return ts;
}

// -----------------------------------------------------------------------------
// Private - parse_type_string
// Unchanged from original implementation.
// -----------------------------------------------------------------------------

bool RosMessageConverter::parse_type_string(
  const std::string & type_string,
  std::string & package,
  std::string & subfolder,
  std::string & type_name)
{
  size_t first_slash = type_string.find('/');
  if (first_slash == std::string::npos) { return false; }

  size_t second_slash = type_string.find('/', first_slash + 1);
  if (second_slash == std::string::npos) { return false; }

  package   = type_string.substr(0, first_slash);
  subfolder = type_string.substr(first_slash + 1, second_slash - first_slash - 1);
  type_name = type_string.substr(second_slash + 1);

  return !package.empty() && !subfolder.empty() && !type_name.empty();
}

// -----------------------------------------------------------------------------
// Private - to_rosx_type
//
// Converts "geometry_msgs/msg/Point" -> "geometry_msgs/Point".
// rosx_introspection ROSType does not use the /msg/ /srv/ subfolder component.
// -----------------------------------------------------------------------------

std::string RosMessageConverter::to_rosx_type(const std::string & type_string)
{
  std::string package, subfolder, type_name;
  if (!parse_type_string(type_string, package, subfolder, type_name))
  {
    return type_string;  // Pass through unparseable strings unchanged
  }
  return package + "/" + type_name;
}

// -----------------------------------------------------------------------------
// Public - members_to_json
// Unchanged from original implementation.
// -----------------------------------------------------------------------------

nlohmann::json RosMessageConverter::members_to_json(
  const rosidl_typesupport_introspection_cpp::MessageMembers * members,
  const uint8_t * data)
{
  using namespace rosidl_typesupport_introspection_cpp;

  nlohmann::json result;

  for (uint32_t i = 0; i < members->member_count_; ++i)
  {
    const MessageMember & member = members->members_[i];
    const uint8_t * field_ptr = data + member.offset_;
    std::string name(member.name_);

    auto read_scalar = [&](const uint8_t * ptr) -> nlohmann::json
    {
      switch (member.type_id_)
      {
        case ROS_TYPE_FLOAT:
          return *reinterpret_cast<const float *>(ptr);
        case ROS_TYPE_DOUBLE:
          return *reinterpret_cast<const double *>(ptr);
        case ROS_TYPE_LONG_DOUBLE:
          return static_cast<double>(*reinterpret_cast<const long double *>(ptr));
        case ROS_TYPE_BOOLEAN:
          return *reinterpret_cast<const bool *>(ptr);
        case ROS_TYPE_CHAR:
        case ROS_TYPE_OCTET:
        case ROS_TYPE_UINT8:
          return *reinterpret_cast<const uint8_t *>(ptr);
        case ROS_TYPE_INT8:
          return *reinterpret_cast<const int8_t *>(ptr);
        case ROS_TYPE_UINT16:
          return *reinterpret_cast<const uint16_t *>(ptr);
        case ROS_TYPE_INT16:
          return *reinterpret_cast<const int16_t *>(ptr);
        case ROS_TYPE_UINT32:
          return *reinterpret_cast<const uint32_t *>(ptr);
        case ROS_TYPE_INT32:
          return *reinterpret_cast<const int32_t *>(ptr);
        case ROS_TYPE_UINT64:
          return *reinterpret_cast<const uint64_t *>(ptr);
        case ROS_TYPE_INT64:
          return *reinterpret_cast<const int64_t *>(ptr);
        case ROS_TYPE_STRING:
          return *reinterpret_cast<const std::string *>(ptr);
        case ROS_TYPE_MESSAGE:
        {
          const auto * sub_members =
            reinterpret_cast<const MessageMembers *>(member.members_->data);
          return members_to_json(sub_members, ptr);
        }
        default:
          return nullptr;
      }
    };

    if (member.is_array_)
    {
      nlohmann::json arr = nlohmann::json::array();
      size_t count = member.array_size_;
      if (member.is_upper_bound_ || count == 0)
      {
        count = member.size_function ? member.size_function(field_ptr) : 0;
      }
      for (size_t j = 0; j < count; ++j)
      {
        const void * elem = member.get_const_function
          ? member.get_const_function(field_ptr, j)
          : nullptr;
        if (elem)
        {
          arr.push_back(read_scalar(reinterpret_cast<const uint8_t *>(elem)));
        }
      }
      result[name] = arr;
    }
    else
    {
      result[name] = read_scalar(field_ptr);
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// Public - json_to_members
// Unchanged from original implementation.
// -----------------------------------------------------------------------------

bool RosMessageConverter::json_to_members(
  const rosidl_typesupport_introspection_cpp::MessageMembers * members,
  const nlohmann::json & json,
  uint8_t * data)
{
  using namespace rosidl_typesupport_introspection_cpp;

  for (uint32_t i = 0; i < members->member_count_; ++i)
  {
    const MessageMember & member = members->members_[i];
    uint8_t * field_ptr = data + member.offset_;
    std::string name(member.name_);

    if (!json.contains(name)) { continue; }

    const nlohmann::json & val = json[name];

    auto write_scalar = [&](uint8_t * ptr, const nlohmann::json & v) -> bool
    {
      try
      {
        switch (member.type_id_)
        {
          case ROS_TYPE_FLOAT:
            *reinterpret_cast<float *>(ptr) = v.get<float>();
            break;
          case ROS_TYPE_DOUBLE:
            *reinterpret_cast<double *>(ptr) = v.get<double>();
            break;
          case ROS_TYPE_LONG_DOUBLE:
            *reinterpret_cast<long double *>(ptr) = v.get<double>();
            break;
          case ROS_TYPE_BOOLEAN:
            *reinterpret_cast<bool *>(ptr) = v.get<bool>();
            break;
          case ROS_TYPE_CHAR:
          case ROS_TYPE_UINT8:
            *reinterpret_cast<uint8_t *>(ptr) = v.get<uint8_t>();
            break;
          case ROS_TYPE_INT8:
            *reinterpret_cast<int8_t *>(ptr) = v.get<int8_t>();
            break;
          case ROS_TYPE_UINT16:
            *reinterpret_cast<uint16_t *>(ptr) = v.get<uint16_t>();
            break;
          case ROS_TYPE_INT16:
            *reinterpret_cast<int16_t *>(ptr) = v.get<int16_t>();
            break;
          case ROS_TYPE_UINT32:
            *reinterpret_cast<uint32_t *>(ptr) = v.get<uint32_t>();
            break;
          case ROS_TYPE_INT32:
            *reinterpret_cast<int32_t *>(ptr) = v.get<int32_t>();
            break;
          case ROS_TYPE_UINT64:
            *reinterpret_cast<uint64_t *>(ptr) = v.get<uint64_t>();
            break;
          case ROS_TYPE_INT64:
            *reinterpret_cast<int64_t *>(ptr) = v.get<int64_t>();
            break;
          case ROS_TYPE_STRING:
            *reinterpret_cast<std::string *>(ptr) = v.get<std::string>();
            break;
          case ROS_TYPE_MESSAGE:
          {
            const auto * sub_members =
              reinterpret_cast<const MessageMembers *>(member.members_->data);
            return json_to_members(sub_members, v, ptr);
          }
          default:
            return false;
        }
        return true;
      }
      catch (const std::exception &)
      {
        return false;
      }
    };

    if (member.is_array_)
    {
      if (!val.is_array()) { return false; }
      size_t count = val.size();
      if ((member.is_upper_bound_ || member.array_size_ == 0) && member.resize_function)
      {
        member.resize_function(field_ptr, count);
      }
      else
      {
        count = std::min(count, member.array_size_);
      }
      for (size_t j = 0; j < count; ++j)
      {
        void * elem = member.get_function ? member.get_function(field_ptr, j) : nullptr;
        if (elem)
        {
          if (!write_scalar(reinterpret_cast<uint8_t *>(elem), val[j]))
          {
            return false;
          }
        }
      }
    }
    else
    {
      if (!write_scalar(field_ptr, val)) { return false; }
    }
  }

  return true;
}
