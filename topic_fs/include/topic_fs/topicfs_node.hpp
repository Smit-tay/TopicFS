#ifndef TOPICFS_NODE_HPP
#define TOPICFS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <map>
#include <vector>
#include <set>
#include <mutex>
#include <chrono>

// #define FUSE_USE_VERSION 31  // required here because of use of : fuse* handle
#include <fuse3/fuse.h>

class topicfsNode : public rclcpp::Node
{
public:
  topicfsNode();

  std::unordered_map<std::string, std::string> latest_messages;
  std::mutex messages_mutex;
  std::unordered_map<std::string, uint64_t> message_versions_;
  std::vector<std::string> writable_topics_;
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;
  std::map<std::string, std::string> topic_types_;
  void set_fuse_handle(fuse* handle) { fuse_handle_ = handle; }
  void set_mount_point(const std::string& mount_point) { mount_point_ = mount_point; }
  void set_notification_interval(std::chrono::milliseconds interval) { notification_interval_ = interval; }
  void subscribe_to_topic(const std::string& topic_name, const std::string& topic_type);
  std::vector<std::string> get_topics();

private:
  fuse* fuse_handle_{nullptr};
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_notification_;
  std::mutex notification_mutex_;
  std::string mount_point_;
  std::chrono::milliseconds notification_interval_{100}; // Default 100ms
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  
  void discover_topics();
  void notify_file_change(const std::string& topic, fuse* fuse_handle);
};

extern std::shared_ptr<topicfsNode> ros2_node;

#endif // TOPICFS_NODE_HPP
