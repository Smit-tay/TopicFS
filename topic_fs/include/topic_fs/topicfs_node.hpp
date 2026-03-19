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

#ifndef TOPICFS_NODE_HPP
#define TOPICFS_NODE_HPP

// Standard library
#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

// Third-party
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

// FUSE - kept separate due to version requirements
#include <fuse3/fuse.h>

class topicfsNode : public rclcpp::Node
{
public:
  topicfsNode();

  // Accessors - topic data
  std::optional<std::string> get_latest_message(const std::string& topic);
  std::optional<std::string> get_topic_type(const std::string& topic);
  uint64_t get_message_version(const std::string& topic);
  std::vector<std::string> get_topics();

  // Accessors - publishers
  bool has_publisher(const std::string& topic);
  bool publish_message(const std::string& topic, rclcpp::SerializedMessage& msg);

  // Configuration
  void set_fuse_handle(fuse* handle);
  void set_mount_point(const std::string& mount_point);
  void set_notification_interval(std::chrono::milliseconds interval);

  // Topic management
  void subscribe_to_topic(const std::string& topic_name, const std::string& topic_type);

private:
  // Shared data - all access must go through public accessors
  std::mutex messages_mutex_;
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;
  std::map<std::string, std::string> topic_types_;
  std::unordered_map<std::string, std::string> latest_messages_;
  std::unordered_map<std::string, uint64_t> message_versions_;

  // Node state
  fuse* fuse_handle_{nullptr};
  std::chrono::milliseconds notification_interval_{100};
  std::string mount_point_;
  std::vector<std::string> writable_topics_;

  // Discovery timer
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  int discovery_interval_ms_{1000};

  // Notification throttling
  std::mutex notification_mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_notification_;

  // Subscriptions
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;

  // Private methods
  void discover_topics();
  void notify_file_change(const std::string& topic, fuse* fuse_handle);
};

#endif // TOPICFS_NODE_HPP
