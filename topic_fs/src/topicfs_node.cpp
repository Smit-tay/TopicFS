// topicfs_node.cpp
// Copyright 2025 Jack Sidman Smith
//
// Licensed under the MIT License. See LICENSE in project root.

// Standard library
#include <algorithm>
#include <chrono>
#include <set>
#include <string>
#include <vector>

// Third-party
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_action/graph.h>

// FUSE
#include <fuse3/fuse.h>
#include <fuse_lowlevel.h>

// Project
#include "topic_fs/topicfs_node.hpp"

// -----------------------------------------------------------------------------
// Local helpers
// -----------------------------------------------------------------------------

// Fallback encoder for message types whose typesupport .so is not available
// at runtime. The raw CDR bytes are base64-encoded and stored as JSON so that
// the data is at least accessible, even if not human-readable.
// Once the correct .so is added to AMENT_PREFIX_PATH and setup.bash sourced,
// the next received message will be stored as proper JSON instead.
static std::string base64_encode(const uint8_t* data, size_t length)
{
  static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string encoded;
  encoded.reserve((length + 2) / 3 * 4);

  for (size_t i = 0; i < length; i += 3)
  {
    uint32_t b = (static_cast<uint32_t>(data[i]) << 16) |
                 ((i + 1 < length ? static_cast<uint32_t>(data[i + 1]) : 0u) << 8) |
                  (i + 2 < length ? static_cast<uint32_t>(data[i + 2]) : 0u);

    encoded.push_back(base64_chars[(b >> 18) & 0x3F]);
    encoded.push_back(base64_chars[(b >> 12) & 0x3F]);
    encoded.push_back(i + 1 < length ? base64_chars[(b >> 6) & 0x3F] : '=');
    encoded.push_back(i + 2 < length ? base64_chars[b        & 0x3F] : '=');
  }

  return encoded;
}

// Returns true if a service or topic name contains the ROS2 action internal
// sub-path "/_action/". Used to filter raw action primitives from the topic
// and service discovery paths — they are instead handled by discover_actions().
static bool is_action_internal(const std::string& name)
{
  return name.find("/_action/") != std::string::npos;
}

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------

topicfsNode::topicfsNode() : Node("topicfs_node")
{
  declare_parameter<std::vector<std::string>>("writable_topics",
                                              std::vector<std::string>{});
  declare_parameter<int>("notification_interval_ms", 100);
  declare_parameter<int>("discovery_interval_ms", 10000);

  // writable_topics can be passed as a string array or (for convenience on the
  // command line) as a single string.  Handle both.
  try
  {
    get_parameter("writable_topics", writable_topics_);
  }
  catch (const rclcpp::ParameterTypeException&)
  {
    std::string single;
    try
    {
      get_parameter("writable_topics", single);
      if (!single.empty())
      {
        writable_topics_ = {single};
        RCLCPP_INFO(get_logger(),
                    "writable_topics: converted single string '%s' to array",
                    single.c_str());
      }
    }
    catch (const rclcpp::ParameterTypeException& e2)
    {
      RCLCPP_ERROR(get_logger(),
                   "writable_topics: invalid type — %s", e2.what());
      throw;
    }
  }

  int interval_ms = 100;
  get_parameter("notification_interval_ms", interval_ms);
  notification_interval_ = std::chrono::milliseconds(interval_ms);

  get_parameter("discovery_interval_ms", discovery_interval_ms_);

  RCLCPP_INFO(get_logger(), "Notification interval : %d ms", interval_ms);
  RCLCPP_INFO(get_logger(), "Discovery interval    : %d ms", discovery_interval_ms_);
  RCLCPP_INFO(get_logger(), "Writable topics       : %zu", writable_topics_.size());
  for (const auto& t : writable_topics_)
  {
    RCLCPP_INFO(get_logger(), "  writable: %s", t.c_str());
  }

  discovery_timer_ = create_wall_timer(
    std::chrono::milliseconds(discovery_interval_ms_),
    [this]()
    {
      RCLCPP_INFO(get_logger(), "discover: timer tick");
      discover_actions();   // must run before topics/services so filters work
      discover_topics();
      discover_services();
    });
}

// -----------------------------------------------------------------------------
// Public accessors — topic data
// -----------------------------------------------------------------------------

std::optional<std::string> topicfsNode::get_latest_message(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  auto it = latest_messages_.find(topic);
  if (it == latest_messages_.end()) { return std::nullopt; }
  return it->second;
}

std::optional<std::string> topicfsNode::get_topic_type(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  auto it = topic_types_.find(topic);
  if (it == topic_types_.end()) { return std::nullopt; }
  return it->second;
}

uint64_t topicfsNode::get_message_version(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  auto it = message_versions_.find(topic);
  if (it == message_versions_.end()) { return 0; }
  return it->second;
}

std::vector<std::string> topicfsNode::get_topics()
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  std::vector<std::string> result;
  result.reserve(topic_types_.size());
  for (const auto& [topic, _] : topic_types_)
  {
    // Strip leading slash — FUSE paths are relative to the mount root
    result.push_back(topic.empty() || topic[0] != '/' ? topic : topic.substr(1));
  }
  return result;
}

// -----------------------------------------------------------------------------
// Public accessors — publishers
// -----------------------------------------------------------------------------

bool topicfsNode::has_publisher(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  return publishers_.count(topic) > 0;
}

bool topicfsNode::publish_message(const std::string& topic,
                                  rclcpp::SerializedMessage& msg)
{
  std::lock_guard<std::mutex> lock(messages_mutex_);
  auto it = publishers_.find(topic);
  if (it == publishers_.end() || !it->second) { return false; }
  it->second->publish(msg);
  return true;
}

// -----------------------------------------------------------------------------
// Public accessors — poll handles
// -----------------------------------------------------------------------------

void topicfsNode::store_poll_handle(const std::string& topic, fuse_pollhandle* ph)
{
  std::lock_guard<std::mutex> lock(poll_mutex_);
  auto it = poll_handles_.find(topic);
  if (it != poll_handles_.end() && it->second)
  {
    // Destroy any handle that was stored but never consumed
    fuse_pollhandle_destroy(it->second);
  }
  poll_handles_[topic] = ph;
}

fuse_pollhandle* topicfsNode::take_poll_handle(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(poll_mutex_);
  auto it = poll_handles_.find(topic);
  if (it == poll_handles_.end()) { return nullptr; }
  fuse_pollhandle* ph = it->second;
  it->second = nullptr;
  return ph;
}

// -----------------------------------------------------------------------------
// Public accessors — services
// -----------------------------------------------------------------------------

std::vector<std::string> topicfsNode::get_services()
{
  std::lock_guard<std::mutex> lock(services_mutex_);
  std::vector<std::string> result;
  result.reserve(service_types_.size());
  for (const auto& [service, _] : service_types_)
  {
    result.push_back(
      service.empty() || service[0] != '/' ? service : service.substr(1));
  }
  return result;
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
  if (it == last_responses_.end()) { return std::nullopt; }
  return it->second;
}

// -----------------------------------------------------------------------------
// Public accessors — actions
// -----------------------------------------------------------------------------

std::vector<std::string> topicfsNode::get_actions()
{
  std::lock_guard<std::mutex> lock(services_mutex_);
  std::vector<std::string> result;
  result.reserve(known_actions_.size());
  for (const auto& [name, _] : known_actions_)
  {
    // Strip leading slash — FUSE paths are relative to the mount root
    result.push_back(name.empty() || name[0] != '/' ? name : name.substr(1));
  }
  return result;
}

bool topicfsNode::has_action(const std::string& action)
{
  std::string key = (action.empty() || action[0] != '/') ? "/" + action : action;
  std::lock_guard<std::mutex> lock(services_mutex_);
  return known_actions_.count(key) > 0;
}

std::optional<ActionEntry> topicfsNode::get_action_entry(const std::string& action)
{
  std::string key = (action.empty() || action[0] != '/') ? "/" + action : action;
  std::lock_guard<std::mutex> lock(services_mutex_);
  auto it = known_actions_.find(key);
  if (it == known_actions_.end()) { return std::nullopt; }
  return it->second;
}

// -----------------------------------------------------------------------------
// Public — call_service
//
// Sends a JSON request to a ROS2 service and returns the JSON response.
// Blocks the calling thread (a FUSE worker thread) for up to 10 seconds.
// The ROS2 executor spinning in ros_thread_ processes the response while
// this function polls the future.
// -----------------------------------------------------------------------------

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
      return R"({"error":"service not found"})";
    }
    client       = it->second;
    service_type = service_types_[service];
  }

  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    return R"({"error":"service not available"})";
  }

  // Build and fill the request message from JSON.
  // Service request structs are generated by ROS2 as ordinary message structs
  // and share the same typesupport/introspection conventions, so we use
  // RosMessageConverter::get_type_support with the "_Request" suffix.
  std::string request_type = service_type + "_Request";

  const rosidl_message_type_support_t* ts =
    RosMessageConverter::get_type_support(request_type);
  if (!ts)
  {
    return R"({"error":"failed to load request typesupport — is the package .so on AMENT_PREFIX_PATH?"})";
  }

  const rosidl_message_type_support_t* intr_ts =
    get_message_typesupport_handle(ts, "rosidl_typesupport_introspection_cpp");
  if (!intr_ts)
  {
    return R"({"error":"failed to get request introspection typesupport"})";
  }

  const auto* members =
    reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
      intr_ts->data);

  std::vector<uint8_t> req_buf(members->size_of_);
  members->init_function(req_buf.data(),
                         rosidl_runtime_cpp::MessageInitialization::ALL);

  if (!RosMessageConverter::json_to_members(members, request, req_buf.data()))
  {
    members->fini_function(req_buf.data());
    return R"({"error":"failed to fill request from JSON"})";
  }

  // Send — GenericClient takes raw message memory (not CDR)
  auto future_and_id =
    client->async_send_request(static_cast<void*>(req_buf.data()));

  // Poll the future while the ROS2 executor (ros_thread_) processes it.
  // We do not call spin_until_future_complete here because this is already
  // running on a FUSE worker thread, not the ROS2 thread.
  constexpr auto timeout   = std::chrono::seconds(10);
  constexpr auto poll_step = std::chrono::milliseconds(10);
  auto start = std::chrono::steady_clock::now();

  while (future_and_id.future.wait_for(poll_step) != std::future_status::ready)
  {
    if (std::chrono::steady_clock::now() - start > timeout)
    {
      client->remove_pending_request(future_and_id.request_id);
      members->fini_function(req_buf.data());
      return R"({"error":"service call timed out"})";
    }
  }

  members->fini_function(req_buf.data());

  auto response = future_and_id.future.get();
  if (!response)
  {
    return R"({"error":"null response"})";
  }

  // Deserialize response — same introspection approach as request
  std::string response_type = service_type + "_Response";

  const rosidl_message_type_support_t* resp_ts =
    RosMessageConverter::get_type_support(response_type);
  if (!resp_ts)
  {
    return R"({"error":"failed to load response typesupport"})";
  }

  const rosidl_message_type_support_t* resp_intr_ts =
    get_message_typesupport_handle(resp_ts, "rosidl_typesupport_introspection_cpp");
  if (!resp_intr_ts)
  {
    return R"({"error":"failed to get response introspection typesupport"})";
  }

  const auto* resp_members =
    reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
      resp_intr_ts->data);

  // GenericClient response is raw message memory — deserialize directly
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
// Topic management — subscribe_to_topic
// -----------------------------------------------------------------------------

void topicfsNode::subscribe_to_topic(
  const std::string& topic_name, const std::string& topic_type)
{
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    if (subscriptions_.count(topic_name))
    {
      return;  // Already subscribed — normal during repeated discovery timer ticks
    }
  }

  try
  {
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();

    auto sub = create_generic_subscription(
      topic_name, topic_type, qos,
      [this, topic_name, topic_type](
        std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
      {
        const auto& buffer = serialized_msg->get_rcl_serialized_message();
        if (buffer.buffer_length == 0 || !buffer.buffer)
        {
          RCLCPP_WARN(get_logger(),
                      "subscribe: empty message on %s", topic_name.c_str());
          return;
        }

        // Try human-readable JSON via typesupport introspection.
        // Falls back to base64 if the .so is not yet available — the next
        // message will retry automatically (cache miss triggers dlopen again).
        std::string stored;
        auto json_result = RosMessageConverter::to_json(topic_type, *serialized_msg);
        if (json_result.has_value())
        {
          stored = json_result->dump();
        }
        else
        {
          RCLCPP_DEBUG(get_logger(),
                       "subscribe: introspection unavailable for %s — "
                       "storing base64 (source the package setup.bash to fix)",
                       topic_type.c_str());
          nlohmann::json j;
          j["_encoding"] = "base64_cdr";
          j["_type"]     = topic_type;
          j["data"]      = base64_encode(buffer.buffer, buffer.buffer_length);
          stored = j.dump();
        }

        uint64_t version;
        {
          std::lock_guard<std::mutex> lock(messages_mutex_);
          latest_messages_[topic_name] = stored + "\n";
          version = ++message_versions_[topic_name];
        }

        notify_file_change(topic_name, fuse_handle_);
        RCLCPP_DEBUG(get_logger(), "subscribe: stored %s v%lu",
                     topic_name.c_str(), version);
      });

    {
      std::lock_guard<std::mutex> lock(messages_mutex_);
      subscriptions_[topic_name]   = sub;
      topic_types_[topic_name]     = topic_type;
      message_versions_[topic_name] = 0;

      if (std::find(writable_topics_.begin(), writable_topics_.end(), topic_name)
          != writable_topics_.end())
      {
        publishers_[topic_name] =
          create_generic_publisher(topic_name, topic_type, qos);
        RCLCPP_INFO(get_logger(), "subscribe: publisher created for %s",
                    topic_name.c_str());
      }
    }

    RCLCPP_INFO(get_logger(), "Subscribed: %s  [%s]",
                topic_name.c_str(), topic_type.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "subscribe_to_topic: %s (%s): %s",
                 topic_name.c_str(), topic_type.c_str(), e.what());
  }
}

// -----------------------------------------------------------------------------
// Discovery — actions
//
// Uses rcl_action_get_names_and_types() — the official C API.
// For each discovered action, subscribes to its feedback and status topics
// and creates service clients for send_goal, cancel_goal, and get_result.
// These reuse the existing topic/service infrastructure unchanged.
// Once known, the action's /_action/ primitives are filtered from
// discover_topics() and discover_services() so they don't appear in the
// filesystem as raw entries.
// -----------------------------------------------------------------------------

void topicfsNode::discover_actions()
{
  rcl_names_and_types_t nat = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator  = rcl_get_default_allocator();
  const rcl_ret_t ret = rcl_action_get_names_and_types(
    get_node_base_interface()->get_rcl_node_handle(), &allocator, &nat);
  if (ret != RCL_RET_OK) { return; }

  std::map<std::string, std::vector<std::string>> names_and_types;
  for (size_t i = 0; i < nat.names.size; ++i)
  {
    std::vector<std::string> types;
    for (size_t j = 0; j < nat.types[i].size; ++j)
    {
      types.push_back(nat.types[i].data[j]);
    }
    names_and_types[nat.names.data[i]] = types;
  }
   if (rcl_names_and_types_fini(&nat) != RCL_RET_OK)
  {
    RCLCPP_WARN(get_logger(), "discover_actions: failed to free names and types");
  }

  for (const auto& [action_name, types] : names_and_types)
  {
    if (types.empty()) { continue; }

    {
      std::lock_guard<std::mutex> lock(services_mutex_);
      if (known_actions_.count(action_name)) { continue; }
    }

    RCLCPP_INFO(get_logger(), "discover_actions: new action %s  [%s]",
                action_name.c_str(), types[0].c_str());

    // Derive the five underlying primitive names from the action name
    const std::string send_goal  = action_name + "/_action/send_goal";
    const std::string cancel     = action_name + "/_action/cancel_goal";
    const std::string get_result = action_name + "/_action/get_result";
    const std::string feedback   = action_name + "/_action/feedback";
    const std::string status     = action_name + "/_action/status";

    // Subscribe to feedback and status topics
    auto all_topics = get_topic_names_and_types();
    for (const auto& [tname, ttypes] : all_topics)
    {
      if ((tname == feedback || tname == status) && !ttypes.empty())
      {
        subscribe_to_topic(tname, ttypes[0]);
      }
    }

    // Create service clients for the three action services
    auto all_services = get_service_names_and_types();
    for (const auto& [sname, stypes] : all_services)
    {
      if ((sname == send_goal || sname == cancel || sname == get_result)
          && !stypes.empty())
      {
        std::lock_guard<std::mutex> lock(services_mutex_);
        if (!service_clients_.count(sname))
        {
          try
          {
            auto client = create_generic_client(sname, stypes[0]);
            service_clients_[sname] = client;
            service_types_[sname]   = stypes[0];
            RCLCPP_INFO(get_logger(),
                        "discover_actions: client created for %s", sname.c_str());
          }
          catch (const std::exception& e)
          {
            RCLCPP_ERROR(get_logger(),
                         "discover_actions: failed client for %s: %s",
                         sname.c_str(), e.what());
          }
        }
      }
    }

    // Register the action entry
    ActionEntry entry;
    entry.action_name = action_name;
    entry.send_goal   = send_goal;
    entry.cancel_goal = cancel;
    entry.get_result  = get_result;
    entry.feedback    = feedback;
    entry.status      = status;

    {
      std::lock_guard<std::mutex> lock(services_mutex_);
      known_actions_[action_name] = entry;
    }
  }
}

// -----------------------------------------------------------------------------
// Discovery — topics
// -----------------------------------------------------------------------------

void topicfsNode::discover_topics()
{
  // ROS2 infrastructure topics — never useful to expose
  static const std::set<std::string> ignored = {
    "/rosout",
    "/parameter_events"
  };

  auto names_and_types = get_topic_names_and_types();
  RCLCPP_DEBUG(get_logger(), "discover_topics: %zu topics visible",
               names_and_types.size());

  for (const auto& [topic, types] : names_and_types)
  {
    if (ignored.count(topic) || types.empty()) { continue; }

    // Action internals are handled by discover_actions() — suppress raw entries
    if (is_action_internal(topic)) { continue; }

    {
      std::lock_guard<std::mutex> lock(messages_mutex_);
      if (subscriptions_.count(topic)) { continue; }
    }

    RCLCPP_INFO(get_logger(), "discover_topics: new topic %s  [%s]",
                topic.c_str(), types[0].c_str());
    subscribe_to_topic(topic, types[0]);
  }
}

// -----------------------------------------------------------------------------
// Discovery — services
// -----------------------------------------------------------------------------

void topicfsNode::discover_services()
{
  // Own-node services — never expose these
  static const std::set<std::string> ignored_prefixes = {
    "/topicfs_node/"
  };

  // Standard per-node ROS2 parameter services — filter by suffix
  static const std::set<std::string> ignored_suffixes = {
    "/describe_parameters",
    "/get_parameter_types",
    "/get_parameters",
    "/get_type_description",
    "/list_parameters",
    "/set_parameters",
    "/set_parameters_atomically"
  };

  auto names_and_types = get_service_names_and_types();
  RCLCPP_DEBUG(get_logger(), "discover_services: %zu services visible",
               names_and_types.size());

  for (const auto& [service, types] : names_and_types)
  {
    if (types.empty()) { continue; }

    bool ignored = false;

    for (const auto& prefix : ignored_prefixes)
    {
      if (service.rfind(prefix, 0) == 0) { ignored = true; break; }
    }
    if (ignored) { continue; }

    for (const auto& suffix : ignored_suffixes)
    {
      if (service.size() >= suffix.size() &&
          service.compare(service.size() - suffix.size(),
                          suffix.size(), suffix) == 0)
      {
        ignored = true;
        break;
      }
    }
    if (ignored) { continue; }

    // Action internals are handled by discover_actions() — suppress raw entries
    if (is_action_internal(service)) { continue; }

    {
      std::lock_guard<std::mutex> lock(services_mutex_);
      if (service_clients_.count(service)) { continue; }
    }

    try
    {
      auto client = create_generic_client(service, types[0]);
      {
        std::lock_guard<std::mutex> lock(services_mutex_);
        service_clients_[service] = client;
        service_types_[service]   = types[0];
      }
      RCLCPP_INFO(get_logger(), "discover_services: new service %s  [%s]",
                  service.c_str(), types[0].c_str());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "discover_services: failed to create client "
                                 "for %s: %s", service.c_str(), e.what());
    }
  }
}

// -----------------------------------------------------------------------------
// Private — notify_file_change
// -----------------------------------------------------------------------------

void topicfsNode::notify_file_change(const std::string& topic, fuse* fuse_handle)
{
  if (!fuse_handle)
  {
    RCLCPP_DEBUG(get_logger(),
                 "notify: FUSE handle not yet set for %s", topic.c_str());
    return;
  }

  auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(notification_mutex_);
    auto& last_time = last_notification_[topic];
    if (now - last_time < notification_interval_)
    {
      RCLCPP_DEBUG(get_logger(), "notify: throttled for %s", topic.c_str());
      return;
    }
    last_time = now;
  }

  // Invalidate the kernel page cache entry for <topic>/latest so that the
  // next read() call hits the FUSE handler rather than returning stale data.
  // The inode number must match what topicfs_getattr() returns for this path.
  struct fuse_session* session = fuse_get_session(fuse_handle);
  ino_t ino = 1 + std::hash<std::string>{}(topic + "/latest");
  int ret = fuse_lowlevel_notify_inval_inode(session, ino, 0, 0);
  if (ret != 0 && ret != -ENOENT)
  {
    RCLCPP_DEBUG(get_logger(),
                 "notify: inval_inode failed for %s: %s",
                 topic.c_str(), strerror(-ret));
  }

  // Wake any poll() / select() waiter (e.g. tail -f) on this topic
  fuse_pollhandle* ph = take_poll_handle(topic);
  if (ph)
  {
    fuse_notify_poll(ph);
    fuse_pollhandle_destroy(ph);
    RCLCPP_DEBUG(get_logger(), "notify: woke poll waiter for %s", topic.c_str());
  }
}
