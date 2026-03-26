// topicfs_fuse.cpp
// Copyright 2025 Jack Sidman Smith
//
// Licensed under the MIT License. See LICENSE in project root.

// Standard library
#include <algorithm>
#include <chrono>
#include <cstring>
#include <set>
#include <string>
#include <vector>

// Project
#include "topic_fs/topicfs.hpp"
#include "topic_fs/topicfs_fuse.hpp"
#include "topic_fs/ros_message_converter.hpp"

// -----------------------------------------------------------------------------
// Local helpers
// -----------------------------------------------------------------------------

static constexpr fuse_fill_dir_flags  zero_fill_dir_flags  = static_cast<fuse_fill_dir_flags>(0);
static constexpr fuse_readdir_flags   zero_readdir_flags   = static_cast<fuse_readdir_flags>(0);

// Decompose a FUSE path into (name, file) components.
// /some/ns/topic/latest  ->  name="some/ns/topic", file="latest"
// Returns false if path has no interior slash (i.e. is root or top-level dir).
static bool split_path(const std::string& spath, std::string& name, std::string& file)
{
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    return false;
  }
  name = spath.substr(1, pos - 1);
  file = spath.substr(pos + 1);
  return true;
}

// -----------------------------------------------------------------------------
// FUSE callbacks
// -----------------------------------------------------------------------------

int topicfs_getattr(const char* path, struct stat* stbuf, struct fuse_file_info* fi)
{
  (void)fi;
  memset(stbuf, 0, sizeof(struct stat));

  auto* topicfs   = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto  ros2_node = topicfs->get_ros2_node();
  std::string spath(path);

  // Root directory
  if (spath == "/")
  {
    stbuf->st_mode  = S_IFDIR | 0755;
    stbuf->st_nlink = 2;
    return 0;
  }

  auto topics   = ros2_node->get_topics();
  auto services = ros2_node->get_services();
  std::string path_no_slash = spath.substr(1);

  stbuf->st_ino = 1 + std::hash<std::string>{}(spath);

  // Check if this path is a directory prefix of any known topic or service
  auto is_dir_prefix = [&](const std::vector<std::string>& names) -> bool
  {
    return std::any_of(names.begin(), names.end(), [&](const auto& n) {
      return n.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
             (n.length() == path_no_slash.length() || n[path_no_slash.length()] == '/');
    });
  };

  if (is_dir_prefix(topics) || is_dir_prefix(services))
  {
    stbuf->st_mode  = S_IFDIR | 0755;
    stbuf->st_nlink = 2;
    return 0;
  }

  std::string name, file;
  if (!split_path(spath, name, file))
  {
    return -ENOENT;
  }

  std::string original = "/" + name;

  // ── Topic files ────────────────────────────────────────────────────────────
  if (std::find(topics.begin(), topics.end(), name) != topics.end())
  {
    if (file == "latest" || file == "info")
    {
      stbuf->st_mode  = S_IFREG | 0444;
      stbuf->st_nlink = 1;
      stbuf->st_mtime = time(nullptr);
      if (file == "latest")
      {
        auto msg = ros2_node->get_latest_message(original);
        if (msg) { stbuf->st_size = static_cast<off_t>(msg->size()); }
      }
      else
      {
        auto type = ros2_node->get_topic_type(original);
        std::string info = "Topic: " + original + "\nType: " +
                           type.value_or("unknown") + "\n";
        stbuf->st_size = static_cast<off_t>(info.size());
      }
      return 0;
    }
    if (file == "command" && ros2_node->has_publisher(original))
    {
      stbuf->st_mode  = S_IFREG | 0222;
      stbuf->st_nlink = 1;
      stbuf->st_size  = 0;
      stbuf->st_mtime = time(nullptr);
      return 0;
    }
  }

  // ── Service files ──────────────────────────────────────────────────────────
  if (std::find(services.begin(), services.end(), name) != services.end())
  {
    if (file == "command")
    {
      stbuf->st_mode  = S_IFREG | 0222;
      stbuf->st_nlink = 1;
      stbuf->st_size  = 0;
      stbuf->st_mtime = time(nullptr);
      return 0;
    }
    if (file == "response")
    {
      stbuf->st_mode  = S_IFREG | 0444;
      stbuf->st_nlink = 1;
      stbuf->st_mtime = time(nullptr);
      auto resp = ros2_node->get_last_response(original);
      stbuf->st_size  = resp ? static_cast<off_t>(resp->size()) : 0;
      return 0;
    }
  }

  RCLCPP_DEBUG(ros2_node->get_logger(), "getattr: path %s not found", spath.c_str());
  return -ENOENT;
}

// -----------------------------------------------------------------------------

int topicfs_readdir(
  const char* path, void* buf, fuse_fill_dir_t filler,
  off_t offset, struct fuse_file_info* fi, enum fuse_readdir_flags flags)
{
  (void)fi;
  (void)flags;
  (void)offset;

  auto* topicfs = static_cast<TopicFS*>(fuse_get_context()->private_data);
  if (!topicfs)
  {
    return -ENOENT;
  }

  auto ros2_node = topicfs->get_ros2_node();
  if (!ros2_node)
  {
    return -ENOMEM;
  }

  std::string spath(path);
  RCLCPP_DEBUG(ros2_node->get_logger(), "readdir: path=%s", spath.c_str());

  auto topics   = ros2_node->get_topics();
  auto services = ros2_node->get_services();

  // ── Root directory ─────────────────────────────────────────────────────────
  if (spath == "/")
  {
    if (filler(buf, ".",  nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "..", nullptr, 0, zero_fill_dir_flags))
    {
      return -ENOMEM;
    }

    std::set<std::string> top_level;
    for (const auto& t : topics)
    {
      size_t pos = t.find('/');
      top_level.insert(pos == std::string::npos ? t : t.substr(0, pos));
    }
    for (const auto& s : services)
    {
      size_t pos = s.find('/');
      top_level.insert(pos == std::string::npos ? s : s.substr(0, pos));
    }
    for (const auto& entry : top_level)
    {
      if (filler(buf, entry.c_str(), nullptr, 0, zero_fill_dir_flags))
      {
        return -ENOMEM;
      }
    }
    return 0;
  }

  std::string path_no_slash = spath.substr(1);

  // ── Exact topic directory ──────────────────────────────────────────────────
  if (std::find(topics.begin(), topics.end(), path_no_slash) != topics.end())
  {
    if (filler(buf, ".",      nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "..",     nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "latest", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "info",   nullptr, 0, zero_fill_dir_flags))
    {
      return -ENOMEM;
    }
    if (ros2_node->has_publisher("/" + path_no_slash))
    {
      if (filler(buf, "command", nullptr, 0, zero_fill_dir_flags))
      {
        return -ENOMEM;
      }
    }
    return 0;
  }

  // ── Exact service directory ────────────────────────────────────────────────
  if (std::find(services.begin(), services.end(), path_no_slash) != services.end())
  {
    if (filler(buf, ".",        nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "..",       nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "command",  nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "response", nullptr, 0, zero_fill_dir_flags))
    {
      return -ENOMEM;
    }
    return 0;
  }

  // ── Namespace directory — collect next path component from all matches ─────
  std::set<std::string> subdirs;

  for (const auto& t : topics)
  {
    if (t.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
        t.length() > path_no_slash.length() &&
        t[path_no_slash.length()] == '/')
    {
      std::string rest = t.substr(path_no_slash.length() + 1);
      size_t pos = rest.find('/');
      subdirs.insert(pos == std::string::npos ? rest : rest.substr(0, pos));
    }
  }
  for (const auto& s : services)
  {
    if (s.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
        s.length() > path_no_slash.length() &&
        s[path_no_slash.length()] == '/')
    {
      std::string rest = s.substr(path_no_slash.length() + 1);
      size_t pos = rest.find('/');
      subdirs.insert(pos == std::string::npos ? rest : rest.substr(0, pos));
    }
  }

  if (!subdirs.empty())
  {
    if (filler(buf, ".",  nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "..", nullptr, 0, zero_fill_dir_flags))
    {
      return -ENOMEM;
    }
    for (const auto& dir : subdirs)
    {
      if (filler(buf, dir.c_str(), nullptr, 0, zero_fill_dir_flags))
      {
        return -ENOMEM;
      }
    }
    return 0;
  }

  RCLCPP_DEBUG(ros2_node->get_logger(), "readdir: path %s not found", spath.c_str());
  return -ENOENT;
}

// -----------------------------------------------------------------------------

int topicfs_open(const char* path, struct fuse_file_info* fi)
{
  auto* topicfs   = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto  ros2_node = topicfs->get_ros2_node();
  std::string spath(path);

  std::string name, file;
  if (!split_path(spath, name, file))
  {
    RCLCPP_ERROR(ros2_node->get_logger(), "open: invalid path %s", spath.c_str());
    return -ENOENT;
  }

  std::string original = "/" + name;
  auto topics   = ros2_node->get_topics();
  auto services = ros2_node->get_services();

  // ── Topic files ────────────────────────────────────────────────────────────
  if (std::find(topics.begin(), topics.end(), name) != topics.end())
  {
    if (file == "latest" || file == "info")
    {
      if ((fi->flags & O_ACCMODE) != O_RDONLY)
      {
        return -EACCES;
      }
      // direct_io: bypass kernel page cache so each read() hits the FUSE
      // handler and poll() sees fresh data immediately.
      fi->direct_io = 1;
      return 0;
    }
    if (file == "command" && ros2_node->has_publisher(original))
    {
      if ((fi->flags & O_ACCMODE) != O_WRONLY &&
          (fi->flags & O_ACCMODE) != O_RDWR)
      {
        return -EACCES;
      }
      return 0;
    }
  }

  // ── Service files ──────────────────────────────────────────────────────────
  if (std::find(services.begin(), services.end(), name) != services.end())
  {
    if (file == "command")
    {
      if ((fi->flags & O_ACCMODE) != O_WRONLY &&
          (fi->flags & O_ACCMODE) != O_RDWR)
      {
        return -EACCES;
      }
      return 0;
    }
    if (file == "response")
    {
      if ((fi->flags & O_ACCMODE) != O_RDONLY)
      {
        return -EACCES;
      }
      fi->direct_io = 1;
      return 0;
    }
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "open: path %s not found", spath.c_str());
  return -ENOENT;
}

// -----------------------------------------------------------------------------

int topicfs_poll(
  const char* path, struct fuse_file_info* fi,
  struct fuse_pollhandle* ph, unsigned* reventsp)
{
  (void)fi;
  *reventsp = 0;

  auto* topicfs   = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto  ros2_node = topicfs->get_ros2_node();
  std::string spath(path);

  std::string name, file;
  if (!split_path(spath, name, file))
  {
    return -ENOENT;
  }

  // Non-pollable files — signal data always ready so callers do not block
  if (file != "latest")
  {
    *reventsp = POLLIN;
    return 0;
  }

  std::string original = "/" + name;

  // If a message is already available, signal immediately
  auto msg = ros2_node->get_latest_message(original);
  if (msg)
  {
    *reventsp = POLLIN;
  }

  // Store poll handle so notify_file_change() can wake this waiter.
  // FUSE transfers ownership of ph to us — we must destroy it after use.
  if (ph)
  {
    ros2_node->store_poll_handle(original, ph);
  }

  RCLCPP_DEBUG(ros2_node->get_logger(),
               "poll: %s, data_ready=%s",
               original.c_str(), msg ? "yes" : "no");
  return 0;
}

// -----------------------------------------------------------------------------

int topicfs_read(
  const char* path, char* buf, size_t size, off_t offset, struct fuse_file_info* fi)
{
  (void)fi;

  auto* topicfs   = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto  ros2_node = topicfs->get_ros2_node();
  std::string spath(path);

  if (offset < 0)
  {
    return -EINVAL;
  }

  std::string name, file;
  if (!split_path(spath, name, file))
  {
    return -ENOENT;
  }

  std::string original = "/" + name;

  // ── latest ─────────────────────────────────────────────────────────────────
  if (file == "latest")
  {
    auto msg = ros2_node->get_latest_message(original);
    if (!msg)
    {
      return -ENOENT;
    }
    const std::string& data = *msg;
    if (static_cast<size_t>(offset) >= data.size())
    {
      return 0;
    }
    size_t len = std::min(size, data.size() - static_cast<size_t>(offset));
    memcpy(buf, data.c_str() + offset, len);
    return static_cast<int>(len);
  }

  // ── info ───────────────────────────────────────────────────────────────────
  if (file == "info")
  {
    auto type = ros2_node->get_topic_type(original);
    std::string info = "Topic: " + original + "\nType: " +
                       type.value_or("unknown") + "\n";
    if (static_cast<size_t>(offset) >= info.size())
    {
      return 0;
    }
    size_t len = std::min(size, info.size() - static_cast<size_t>(offset));
    memcpy(buf, info.c_str() + offset, len);
    return static_cast<int>(len);
  }

  // ── response ───────────────────────────────────────────────────────────────
  if (file == "response")
  {
    auto resp = ros2_node->get_last_response(original);
    static const std::string no_response = R"({"status":"no response yet"})";
    const std::string& data = resp ? *resp : no_response;
    if (static_cast<size_t>(offset) >= data.size())
    {
      return 0;
    }
    size_t len = std::min(size, data.size() - static_cast<size_t>(offset));
    memcpy(buf, data.c_str() + offset, len);
    return static_cast<int>(len);
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "read: path %s not found", spath.c_str());
  return -ENOENT;
}

// -----------------------------------------------------------------------------

int topicfs_write(
  const char* path, const char* buf, size_t size, off_t offset, struct fuse_file_info* fi)
{
  (void)fi;
  (void)offset;  // Writes to command files are always treated as atomic

  auto* topicfs   = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto  ros2_node = topicfs->get_ros2_node();
  std::string spath(path);

  std::string name, file;
  if (!split_path(spath, name, file))
  {
    return -ENOENT;
  }

  if (file != "command")
  {
    return -EROFS;
  }

  std::string original = "/" + name;
  std::string data(buf, size);

  // ── Service command ────────────────────────────────────────────────────────
  if (ros2_node->has_service(original))
  {
    try
    {
      nlohmann::json j = nlohmann::json::parse(data);
      std::string result = ros2_node->call_service(original, j);
      RCLCPP_INFO(ros2_node->get_logger(),
                  "write: service %s response: %s", original.c_str(), result.c_str());
      return static_cast<int>(size);
    }
    catch (const nlohmann::json::parse_error& e)
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "write: JSON parse error for service %s: %s",
                   original.c_str(), e.what());
      return -EINVAL;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "write: service call failed for %s: %s",
                   original.c_str(), e.what());
      return -EIO;
    }
  }

  // ── Topic command ──────────────────────────────────────────────────────────
  if (!ros2_node->has_publisher(original))
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "write: no publisher or service for %s", original.c_str());
    return -ENOENT;
  }

  try
  {
    nlohmann::json j = nlohmann::json::parse(data);

    auto topic_type = ros2_node->get_topic_type(original);
    if (!topic_type)
    {
      return -EINVAL;
    }

    auto serialized = RosMessageConverter::from_json(*topic_type, j);
    if (!serialized)
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "write: failed to serialize JSON for %s", original.c_str());
      return -EINVAL;
    }

    if (!ros2_node->publish_message(original, *serialized))
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "write: failed to publish to %s", original.c_str());
      return -EIO;
    }

    return static_cast<int>(size);
  }
  catch (const nlohmann::json::parse_error& e)
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "write: JSON parse error for %s: %s", original.c_str(), e.what());
    return -EINVAL;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "write: exception for %s: %s", original.c_str(), e.what());
    return -EIO;
  }
}

// -----------------------------------------------------------------------------
// Filesystem listing utility
// -----------------------------------------------------------------------------

static int collect_entries(
  void* buf, const char* name, const struct stat* stbuf,
  off_t off, enum fuse_fill_dir_flags flags)
{
  (void)stbuf;
  (void)off;
  (void)flags;
  static_cast<std::vector<std::string>*>(buf)->push_back(name);
  return 0;
}

void list_fuse_filesystem(rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "TopicFS filesystem layout:");
  std::vector<std::string> root_entries;
  if (topicfs_readdir("/", &root_entries, collect_entries, 0, nullptr,
                      zero_readdir_flags) != 0)
  {
    RCLCPP_ERROR(logger, "list: failed to read root");
    return;
  }

  for (const auto& entry : root_entries)
  {
    if (entry == "." || entry == "..") { continue; }
    RCLCPP_INFO(logger, "  /%s", entry.c_str());
    std::string t_path = "/" + entry;
    std::vector<std::string> sub_entries;
    if (topicfs_readdir(t_path.c_str(), &sub_entries, collect_entries, 0,
                        nullptr, zero_readdir_flags) == 0)
    {
      for (const auto& sub : sub_entries)
      {
        if (sub == "." || sub == "..") { continue; }
        RCLCPP_INFO(logger, "    /%s/%s", entry.c_str(), sub.c_str());
      }
    }
  }
}

// -----------------------------------------------------------------------------
// FUSE operations initialisation
// -----------------------------------------------------------------------------

void init_fuse_operations(struct fuse_operations& ops)
{
  memset(&ops, 0, sizeof(struct fuse_operations));
  ops.getattr = topicfs_getattr;
  ops.readdir = topicfs_readdir;
  ops.open    = topicfs_open;
  ops.poll    = topicfs_poll;
  ops.read    = topicfs_read;
  ops.write   = topicfs_write;
}
