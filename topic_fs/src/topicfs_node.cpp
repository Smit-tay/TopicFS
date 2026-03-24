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

// Standard library
#include <algorithm>
#include <chrono>
#include <set>
#include <string>
#include <vector>

// Third-party
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

// FUSE - kept separate due to version requirements
#include <fuse3/fuse.h>
#include <fuse_lowlevel.h>

// Project
#include "topic_fs/topicfs_node.hpp"

// -----------------------------------------------------------------------------
// Local helpers
// -----------------------------------------------------------------------------

static std::string base64_encode(const uint8_t* data, size_t length)
{
  static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve((length + 2) / 3 * 4);

  for (size_t i = 0; i < length; i += 3)
  {
    uint32_t b = (data[i] << 16) |
                 ((i + 1 < length ? data[i + 1] : 0) << 8) |
                 (i + 2 < length ? data[i + 2] : 0);
    encoded.push_back(base64_chars[(b >> 18) & 0x3F]);
    encoded.push_back(base64_chars[(b >> 12) & 0x3F]);
    encoded.push_back(i + 1 < length ? base64_chars[(b >> 6) & 0x3F] : '=');
    encoded.push_back(i + 2 < length ? base64_chars[b & 0x3F] : '=');
  }

  return encoded;
}

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------

topicfsNode::topicfsNode() : Node("ros2_fuse_node")
{
  declare_parameter<std::vector<std::string>>("writable_topics", std::vector<std::string>{});
  declare_parameter<int>("notification_interval_ms", 100);
  declare_parameter<int>("discovery_interval_ms", 1000);

  try
  {
    get_parameter("writable_topics", writable_topics_);

    int interval_ms;
    get_parameter("notification_interval_ms", interval_ms);
    notification_interval_ = std::chrono::milliseconds(interval_ms);
    RCLCPP_INFO(this->get_logger(), "Notification interval set to %d ms", interval_ms);

    get_parameter("discovery_interval_ms", discovery_interval_ms_);
    RCLCPP_INFO(this->get_logger(), "Discovery interval set to %d ms", discovery_interval_ms_);
    discovery_timer_ = create_wall_timer(
        std::chrono::milliseconds(discovery_interval_ms_),
        [this]() {
                discover_topics();
                discover_services();
            });

  }
  catch (const rclcpp::ParameterTypeException& e)
  {
    std::string single_topic;
    try
    {
      declare_parameter<std::string>("writable_topics", "");
      get_parameter("writable_topics", single_topic);
      if (!single_topic.empty())
      {
        writable_topics_ = {single_topic};
        RCLCPP_INFO(this->get_logger(), "Converted single topic '%s' to array",
                    single_topic.c_str());
      }
    }
    catch (const rclcpp::ParameterTypeException& e2)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid type for 'writable_topics': %s. Expected string or string array.",
                   e2.what());
      throw;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Writable topics: %zu", writable_topics_.size());
  for (const auto& topic : writable_topics_)
  {
    RCLCPP_INFO(this->get_logger(), "  - %s", topic.c_str());
  }
}

// -----------------------------------------------------------------------------
// Public accessors - topic data
// -----------------------------------------------------------------------------

std::optional<std::string> topicfsNode::get_latest_message(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  auto it = latest_messages_.find(topic);
  if (it == latest_messages_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

std::optional<std::string> topicfsNode::get_topic_type(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  auto it = topic_types_.find(topic);
  if (it == topic_types_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

uint64_t topicfsNode::get_message_version(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  auto it = message_versions_.find(topic);
  if (it == message_versions_.end())
  {
    return 0;
  }
  return it->second;
}

std::vector<std::string> topicfsNode::get_topics()
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  std::vector<std::string> topics;
  for (const auto& pair : topic_types_)
  {
    std::string topic = pair.first;
    if (!topic.empty() && topic[0] == '/')
    {
      topic = topic.substr(1);
    }
    topics.push_back(topic);
  }
  return topics;
}

// -----------------------------------------------------------------------------
// Public accessors - publishers
// -----------------------------------------------------------------------------

bool topicfsNode::has_publisher(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  return publishers_.count(topic) > 0;
}

bool topicfsNode::publish_message(const std::string& topic, rclcpp::SerializedMessage& msg)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  auto it = publishers_.find(topic);
  if (it == publishers_.end() || !it->second)
  {
    return false;
  }
  it->second->publish(msg);
  return true;
}

// -----------------------------------------------------------------------------
// Public accessors - poll handles
// -----------------------------------------------------------------------------

void topicfsNode::store_poll_handle(const std::string& topic, fuse_pollhandle* ph)
{
  std::lock_guard<std::mutex> lock(poll_mutex_);
  auto it = poll_handles_.find(topic);
  if (it != poll_handles_.end() && it->second)
  {
    // Destroy any previously stored handle that was never consumed
    fuse_pollhandle_destroy(it->second);
  }
  poll_handles_[topic] = ph;
}

fuse_pollhandle* topicfsNode::take_poll_handle(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(poll_mutex_);
  auto it = poll_handles_.find(topic);
  if (it == poll_handles_.end())
  {
    return nullptr;
  }
  fuse_pollhandle* ph = it->second;
  it->second = nullptr;
  return ph;
}

// -----------------------------------------------------------------------------
// Public accessors - services
// -----------------------------------------------------------------------------

std::vector<std::string> topicfsNode::get_services()
{
  std::lock_guard<std::mutex> lock(services_mutex_);
  std::vector<std::string> services;
  for (const auto& pair : service_types_)
  {
    std::string service = pair.first;
    if (!service.empty() && service[0] == '/')
    {
      service = service.substr(1);
    }
    services.push_back(service);
  }
  return services;
}

bool topicfsNode::has_service(const std::string& service)
{
  std::lock_guard<std::mutex> lock(services_mutex_);
  return service_clients_.count(service) > 0;
}

std::optional<std::string> topicfsNode::get_last_response(const std::string& service)
{
  std::lock_guard<std::mutex> lock(services_mutex_);
  auto it = last_responses_.find(service);
  if (it == last_responses_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

std::string topicfsNode::call_service(
  const std::string& service, const nlohmann::json& request)
{
  rclcpp::GenericClient::SharedPtr client;
  std::string service_type;
  {
    std::lock_guard<std::mutex> lock(services_mutex_);
    auto it = service_clients_.find(service);
    if (it == service_clients_.end())
    {
      return R"({"error": "service not found"})";
    }
    client = it->second;
    service_type = service_types_[service];
  }

  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    return R"({"error": "service not available"})";
  }

  // GenericClient::Request is void* pointing to raw message memory (not CDR).
  // We must allocate the message struct, fill it from JSON, and pass the pointer.
  std::string request_type = service_type + "_Request";

  const rosidl_message_type_support_t* ts =
    RosMessageConverter::get_type_support(request_type);
  if (!ts)
  {
    return R"({"error": "failed to load request type support"})";
  }

  const rosidl_message_type_support_t* introspection_ts =
    get_message_typesupport_handle(ts, "rosidl_typesupport_introspection_cpp");
  if (!introspection_ts)
  {
    return R"({"error": "failed to get introspection type support"})";
  }

  const auto* members =
    reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
    introspection_ts->data);

  // Allocate and initialize request message memory
  std::vector<uint8_t> request_buffer(members->size_of_);
  members->init_function(request_buffer.data(),
                         rosidl_runtime_cpp::MessageInitialization::ALL);

  // Fill from JSON
  if (!RosMessageConverter::json_to_members(members, request, request_buffer.data()))
  {
    members->fini_function(request_buffer.data());
    return R"({"error": "failed to fill request from JSON"})";
  }

  // Send request — pass raw message memory pointer
  auto future_and_id = client->async_send_request(
    static_cast<void*>(request_buffer.data()));

  // Wait for response — executor is spinning in ros_thread_
  auto timeout = std::chrono::seconds(10);
  auto start = std::chrono::steady_clock::now();
  while (future_and_id.future.wait_for(std::chrono::milliseconds(10)) !=
         std::future_status::ready)
  {
    if (std::chrono::steady_clock::now() - start > timeout)
    {
      client->remove_pending_request(future_and_id.request_id);
      members->fini_function(request_buffer.data());
      return R"({"error": "service call timed out"})";
    }
  }

  members->fini_function(request_buffer.data());

  auto response = future_and_id.future.get();
  if (!response)
  {
    return R"({"error": "null response"})";
  }

  // GenericClient response is also raw message memory (not CDR).
  // Use introspection to deserialize directly from the response memory.
  std::string response_type = service_type + "_Response";

  const rosidl_message_type_support_t* resp_ts =
    RosMessageConverter::get_type_support(response_type);
  if (!resp_ts)
  {
    return R"({"error": "failed to load response type support"})";
  }

  const rosidl_message_type_support_t* resp_introspection_ts =
    get_message_typesupport_handle(resp_ts, "rosidl_typesupport_introspection_cpp");
  if (!resp_introspection_ts)
  {
    return R"({"error": "failed to get response introspection type support"})";
  }

  const auto* resp_members =
    reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
    resp_introspection_ts->data);

  // Response void* points to raw message memory — deserialize directly
  nlohmann::json json_response = RosMessageConverter::members_to_json(
    resp_members, static_cast<const uint8_t*>(response.get()));

  std::string result = json_response.dump();

  {
    std::lock_guard<std::mutex> lock(services_mutex_);
    last_responses_[service] = result;
  }

  return result;
}

// -----------------------------------------------------------------------------
// Configuration setters
// -----------------------------------------------------------------------------

void topicfsNode::set_fuse_handle(fuse* handle)
{
  fuse_handle_ = handle;
}

void topicfsNode::set_mount_point(const std::string& mount_point)
{
  mount_point_ = mount_point;
}

void topicfsNode::set_notification_interval(std::chrono::milliseconds interval)
{
  notification_interval_ = interval;
}

// -----------------------------------------------------------------------------
// Topic management
// -----------------------------------------------------------------------------

void topicfsNode::subscribe_to_topic(const std::string& topic_name, const std::string& topic_type)
{
    fprintf(stderr, "subscribe_to_topic: captured type='%s' len=%zu\n",
            topic_type.c_str(), topic_type.size());
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    if (subscriptions_.count(topic_name))
    {
      RCLCPP_WARN(this->get_logger(), "Already subscribed to topic: %s", topic_name.c_str());
      return;
    }
  }

  try
  {
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();

    auto sub = create_generic_subscription(
  topic_name, topic_type, qos,
  [this, topic_name, topic_type](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received message on topic: %s", topic_name.c_str());
    const auto& buffer = serialized_msg->get_rcl_serialized_message();
    if (buffer.buffer_length == 0 || !buffer.buffer)
    {
      RCLCPP_WARN(this->get_logger(), "Received empty message on topic: %s",
                  topic_name.c_str());
      return;
    }

    // Attempt human-readable JSON conversion via introspection
    std::string stored;
    auto json_result = RosMessageConverter::to_json(topic_type, *serialized_msg);
    if (json_result.has_value())
    {
      stored = json_result->dump();
    }
    else
    {
      // Fall back to base64 for unknown or custom message types
      RCLCPP_DEBUG(this->get_logger(),
                   "Introspection failed for %s, falling back to base64",
                   topic_type.c_str());
      std::string encoded = base64_encode(buffer.buffer, buffer.buffer_length);
      nlohmann::json j;
      j["data"] = encoded;
      stored = j.dump();
    }

    uint64_t version;
    {
      std::lock_guard<std::mutex> lock(messages_mutex_);
      latest_messages_[topic_name] = stored;
      version = ++message_versions_[topic_name];
    }

    notify_file_change(topic_name, fuse_handle_);
    RCLCPP_DEBUG(this->get_logger(), "Stored message for %s (version %lu)",
                 topic_name.c_str(), version);
  });

    {
      std::lock_guard<std::mutex> lock(messages_mutex_);
      subscriptions_[topic_name] = sub;
      topic_types_[topic_name] = topic_type;
      message_versions_[topic_name] = 0;

      if (std::find(writable_topics_.begin(), writable_topics_.end(), topic_name)
          != writable_topics_.end())
      {
        auto pub = create_generic_publisher(topic_name, topic_type, qos);
        publishers_[topic_name] = pub;
        RCLCPP_INFO(this->get_logger(), "Created publisher for %s (%s)",
                    topic_name.c_str(), topic_type.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to topic: %s (%s) with QoS reliable, volatile, keep-last-10",
                topic_name.c_str(), topic_type.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to topic %s (%s): %s",
                 topic_name.c_str(), topic_type.c_str(), e.what());
  }
}

void topicfsNode::discover_services()
{
  // Skip internal ROS2 infrastructure services
  static const std::set<std::string> ignored_prefixes = {
    "/ros2_fuse_node/",
    "/rosout/"
  };

  auto service_names_and_types = get_service_names_and_types();
  RCLCPP_DEBUG(this->get_logger(), "discover_services: found %zu services",
               service_names_and_types.size());

  for (const auto& [service, types] : service_names_and_types)
  {
    // Skip internal services
    bool ignored = false;
    for (const auto& prefix : ignored_prefixes)
    {
      if (service.rfind(prefix, 0) == 0)
      {
        ignored = true;
        break;
      }
    }
    // Also skip any service ending in standard ROS2 node parameter suffixes
    static const std::set<std::string> ignored_suffixes = {
      "/describe_parameters",
      "/get_parameter_types",
      "/get_parameters",
      "/get_type_description",
      "/list_parameters",
      "/set_parameters",
      "/set_parameters_atomically"
    };
    for (const auto& suffix : ignored_suffixes)
    {
      if (service.size() >= suffix.size() &&
          service.compare(service.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        ignored = true;
        break;
      }
    }
    if (ignored)
    {
      continue;
    }

    if (types.empty())
    {
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(services_mutex_);
      if (service_clients_.count(service))
      {
        continue;
      }
    }

    try
    {
      auto client = create_generic_client(service, types[0]);
      {
        std::lock_guard<std::mutex> lock(services_mutex_);
        service_clients_[service] = client;
        service_types_[service] = types[0];
      }
      RCLCPP_INFO(this->get_logger(), "Discovered service: %s (%s)",
                  service.c_str(), types[0].c_str());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create client for service %s: %s",
                   service.c_str(), e.what());
    }
  }
}

void topicfsNode::discover_topics()
{
  auto topic_names_and_types = get_topic_names_and_types();
  RCLCPP_DEBUG(this->get_logger(), "discover_topics: found %zu topics",
              topic_names_and_types.size());

  // Skip internal ROS2 infrastructure topics
  static const std::set<std::string> ignored_topics = {
    "/rosout",
    "/parameter_events"
  };
             
  std::set<std::string> seen_topics;
  for (const auto& [topic, types] : topic_names_and_types)
  {
      // SKip ignored topics
    if (ignored_topics.count(topic))
    {
      continue;
    }
    if (!types.empty() && seen_topics.insert(topic).second)
    {
      {
        std::lock_guard<std::mutex> lock(messages_mutex_);
        if (subscriptions_.count(topic))
        {
          continue;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Found topic: %s, type: %s",
                  topic.c_str(), types[0].c_str());
      subscribe_to_topic(topic, types[0]);
    }
  }
}

// -----------------------------------------------------------------------------
// Private - notification
// -----------------------------------------------------------------------------

void topicfsNode::notify_file_change(const std::string& topic, fuse* fuse_handle)
{
  if (!fuse_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "notify: no FUSE handle available");
    return;
  }

  auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(notification_mutex_);
    auto& last_time = last_notification_[topic];
    if (now - last_time < notification_interval_)
    {
      RCLCPP_DEBUG(this->get_logger(), "notify: throttled for %s", topic.c_str());
      return;
    }
    last_time = now;
  }

  // Invalidate kernel page cache for the latest file
  struct fuse_session* session = fuse_get_session(fuse_handle);
  ino_t st_ino = 1 + std::hash<std::string>{}("/" + topic.substr(1) + "/latest");
  int ret = fuse_lowlevel_notify_inval_inode(session, st_ino, 0, 0);
  if (ret != 0 && ret != -ENOENT)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "notify: failed to invalidate inode for %s: %s (errno=%d)",
                 topic.c_str(), strerror(-ret), ret);
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "notify: invalidated inode for %s", topic.c_str());
  }

  // Wake any tail -f / poll() waiter on this topic
  fuse_pollhandle* ph = take_poll_handle(topic);
  if (ph)
  {
    fuse_notify_poll(ph);
    fuse_pollhandle_destroy(ph);
    RCLCPP_DEBUG(this->get_logger(), "notify: woke poll waiter for %s", topic.c_str());
  }
}
