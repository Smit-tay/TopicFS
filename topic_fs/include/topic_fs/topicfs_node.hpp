// topicfs_node.hpp
// Copyright 2025 Jack Sidman Smith
//
// Licensed under the MIT License. See LICENSE in project root.

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

// FUSE
#include <fuse3/fuse.h>

// Project
#include "topic_fs/ros_message_converter.hpp"

class topicfsNode : public rclcpp::Node
{
public:
  topicfsNode();

  // ── Topic data ─────────────────────────────────────────────────────────────
  std::optional<std::string> get_latest_message(const std::string& topic);
  std::optional<std::string> get_topic_type(const std::string& topic);
  uint64_t                   get_message_version(const std::string& topic);
  std::vector<std::string>   get_topics();

  // ── Publishers ─────────────────────────────────────────────────────────────
  bool has_publisher(const std::string& topic);
  bool publish_message(const std::string& topic, rclcpp::SerializedMessage& msg);

  // ── Poll handles ───────────────────────────────────────────────────────────
  void             store_poll_handle(const std::string& topic, fuse_pollhandle* ph);
  fuse_pollhandle* take_poll_handle(const std::string& topic);

  // ── Services ───────────────────────────────────────────────────────────────
  std::vector<std::string>   get_services();
  bool                       has_service(const std::string& service);
  std::string                call_service(const std::string& service,
                                          const nlohmann::json& request);
  std::optional<std::string> get_last_response(const std::string& service);

  // ── Configuration ──────────────────────────────────────────────────────────
  void set_fuse_handle(fuse* handle);
  void set_mount_point(const std::string& mount_point);
  void set_notification_interval(std::chrono::milliseconds interval);

  // ── Topic management ───────────────────────────────────────────────────────
  void subscribe_to_topic(const std::string& topic_name, const std::string& topic_type);

private:
  // ── Topic state ────────────────────────────────────────────────────────────
  std::mutex                                                      messages_mutex_;
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr>      publishers_;
  std::map<std::string, std::string>                              topic_types_;
  std::unordered_map<std::string, std::string>                    latest_messages_;
  std::unordered_map<std::string, uint64_t>                       message_versions_;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr>   subscriptions_;

  // ── Poll handles ───────────────────────────────────────────────────────────
  std::mutex                                                       poll_mutex_;
  std::unordered_map<std::string, fuse_pollhandle*>                poll_handles_;

  // ── Service state ──────────────────────────────────────────────────────────
  std::mutex                                                       services_mutex_;
  std::map<std::string, rclcpp::GenericClient::SharedPtr>          service_clients_;
  std::map<std::string, std::string>                               service_types_;
  std::unordered_map<std::string, std::string>                     last_responses_;

  // ── Node state ─────────────────────────────────────────────────────────────
  fuse*                              fuse_handle_{nullptr};
  std::string                        mount_point_;
  std::chrono::milliseconds          notification_interval_{100};
  std::vector<std::string>           writable_topics_;

  // ── Discovery ──────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr       discovery_timer_;
  int                                discovery_interval_ms_{1000};

  // ── Notification throttling ────────────────────────────────────────────────
  std::mutex                                                       notification_mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point>
                                                                   last_notification_;

  // ── Private methods ────────────────────────────────────────────────────────
  void discover_topics();
  void discover_services();

  // Actions are currently exposed as raw /_action/ services and topics —
  // they appear in the filesystem and are interactable but not ergonomic.
  // discover_actions() is reserved for a future task that will implement
  // proper action support using a goal/feedback/result directory structure.
  // At that point, /_action/ internals will be filtered from discover_topics()
  // and discover_services() and replaced by the action abstraction.
  void discover_actions();  // stub — not yet implemented

  void notify_file_change(const std::string& topic, fuse* fuse_handle);
};

#endif // TOPICFS_NODE_HPP
