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

#include "topic_fs/topicfs_node.hpp"
#include "topic_fs/topicfs_fuse.hpp"
// #define FUSE_USE_VERSION 31
#include <fuse3/fuse.h>
#include <fuse_lowlevel.h>

std::string base64_encode(const uint8_t* data, size_t length)
{
  static const std::string base64_chars = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve((length + 2) / 3 * 4);

  for (size_t i = 0; i < length; i += 3)
  {
    uint32_t b = (data[i] << 16) | ((i + 1 < length ? data[i + 1] : 0) << 8) | (i + 2 < length ? data[i + 2] : 0);
    encoded.push_back(base64_chars[(b >> 18) & 0x3F]);
    encoded.push_back(base64_chars[(b >> 12) & 0x3F]);
    encoded.push_back(i + 1 < length ? base64_chars[(b >> 6) & 0x3F] : '=');
    encoded.push_back(i + 2 < length ? base64_chars[b & 0x3F] : '=');
  }

  return encoded;
}

void topicfsNode::notify_file_change(const std::string& topic, fuse* fuse_handle) {
  if (!fuse_handle) {
    RCLCPP_ERROR(this->get_logger(), "notify: no FUSE handle available");
    return;
  }

  auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(notification_mutex_);
  auto& last_time = last_notification_[topic];
  if (now - last_time < notification_interval_) {
    RCLCPP_DEBUG(this->get_logger(), "notify: throttled for %s", topic.c_str());
    return;
  }
  last_time = now;

  std::string path = topic + "/latest";
  RCLCPP_DEBUG(this->get_logger(), "notify: invalidating %s", path.c_str());
  struct fuse_session* session = fuse_get_session(fuse_handle);

  // Look up inode for the file
  // Compute the inode
  ino_t st_ino = 1 + std::hash<std::string>{}("/" + topic.substr(1) + "/latest");
  // struct stat stbuf;
  // if( 0 != topicfs_getattr((path).c_str(), &stbuf, nullptr) ) {
  //   RCLCPP_ERROR(this->get_logger(), "notify: failed to get attributes for %s", path.c_str());
  //   return;
  // }

  // Notify inode change
  //int ret = fuse_lowlevel_notify_inval_inode(session, stbuf.st_ino, 0, 0);
  int ret = fuse_lowlevel_notify_inval_inode(session, st_ino, 0, 0);
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(), "notify: failed to invalidate inode for %s: %s (errno=%d)", 
                 path.c_str(), strerror(-ret), ret);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "notify: successfully invalidated inode for %s", path.c_str());
  }
}

topicfsNode::topicfsNode():Node("ros2_fuse_node")
{
  // Declare writable_topics as a string array
  declare_parameter<std::vector<std::string>>("writable_topics", std::vector<std::string>{});
  declare_parameter<int>("notification_interval_ms", 100);
  
  
  // Try to get writable_topics as an array
  try
  {
    get_parameter("writable_topics", writable_topics_);

    int interval_ms; // needs to be converted to chrono::milliseconds later, type not supported by get_parameter directly
    get_parameter("notification_interval_ms", interval_ms);
    notification_interval_ = std::chrono::milliseconds(interval_ms);
    RCLCPP_INFO(this->get_logger(), "Notification interval set to %d ms", interval_ms);
  }
  catch (const rclcpp::ParameterTypeException& e)
  {
    // If a single string is provided, convert it to a single-element array
    std::string single_topic;
    try
    {
      declare_parameter<std::string>("writable_topics", "");
      get_parameter("writable_topics", single_topic);
      if (!single_topic.empty())
      {
        writable_topics_ = {single_topic};
        RCLCPP_INFO(this->get_logger(), "Converted single topic '%s' to array", single_topic.c_str());
      }
    }
    catch (const rclcpp::ParameterTypeException& e2)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid type for 'writable_topics': %s. Expected string or string array.", e2.what());
      throw;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Writable topics: %zu", writable_topics_.size());
  for (const auto& topic : writable_topics_)
  {
    RCLCPP_INFO(this->get_logger(), "  - %s", topic.c_str());
  }
  discover_topics();
}

void topicfsNode::subscribe_to_topic(const std::string& topic_name, const std::string& topic_type)
{
  if (subscriptions_.count(topic_name))
  {
    RCLCPP_WARN(this->get_logger(), "Already subscribed to topic: %s", topic_name.c_str());
    return;
  }

  try
  {
      rclcpp::QoS qos(10);
      qos.reliable();
      qos.durability_volatile();

      auto sub = create_generic_subscription(
      topic_name, topic_type, qos,
      [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received message on topic: %s", topic_name.c_str());
        const auto& buffer = serialized_msg->get_rcl_serialized_message();
        if (buffer.buffer_length == 0) {
          RCLCPP_WARN(this->get_logger(), "Received empty message on topic: %s", topic_name.c_str());
          return;
        }

        std::string encoded = base64_encode(buffer.buffer, buffer.buffer_length);
        nlohmann::json j;
        j["data"] = encoded;

        std::lock_guard<std::mutex> lock(messages_mutex);
        latest_messages[topic_name] = j.dump();
        message_versions_[topic_name]++;
        notify_file_change(topic_name, fuse_handle_); // TODO: Pass fuse_handle properly
        RCLCPP_DEBUG(this->get_logger(), "Stored message for %s: %s (version %lu)", 
                     topic_name.c_str(), j.dump().c_str(), message_versions_[topic_name]);
      });

    subscriptions_[topic_name] = sub;
    topic_types_[topic_name] = topic_type;
    message_versions_[topic_name] = 0;

    if (std::find(writable_topics_.begin(), writable_topics_.end(), topic_name) != writable_topics_.end())
    {
      auto pub = create_generic_publisher(topic_name, topic_type, qos);
      publishers_[topic_name] = pub;
      RCLCPP_INFO(this->get_logger(), "Created publisher for %s (%s)", topic_name.c_str(), topic_type.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s (%s) with QoS reliable, volatile, keep-last-10", topic_name.c_str(), topic_type.c_str());
    auto sub_info = sub->get_topic_name();
    RCLCPP_INFO(this->get_logger(), "Subscription active for %s with QoS: reliable, volatile", sub_info);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to topic %s (%s): %s", topic_name.c_str(), topic_type.c_str(), e.what());
  }
}

std::vector<std::string> topicfsNode::get_topics()
{
  std::lock_guard<std::mutex> lock(messages_mutex);
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

void topicfsNode::discover_topics()
{
  auto topic_names_and_types = get_topic_names_and_types();
  std::set<std::string> seen_topics;
  for (const auto& [topic, types] : topic_names_and_types)
  {
    if (!types.empty() && seen_topics.insert(topic).second)
    {
      RCLCPP_INFO(this->get_logger(), "Found topic: %s, type: %s", topic.c_str(), types[0].c_str());
      subscribe_to_topic(topic, types[0]);
    }
  }
}