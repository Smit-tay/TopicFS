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
#include <cstring>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// Project
#include "topic_fs/topicfs.hpp"
#include "topic_fs/topicfs_fuse.hpp"
#include "topic_fs/ros_message_converter.hpp"

// -----------------------------------------------------------------------------
// Local helpers
// -----------------------------------------------------------------------------

fuse_fill_dir_flags zero_fill_dir_flags = {static_cast<fuse_fill_dir_flags>(0)};
fuse_readdir_flags zero_readdir_flags = {static_cast<fuse_readdir_flags>(0)};

struct fuse_operations topicfs_oper;

// -----------------------------------------------------------------------------
// FUSE callbacks
// -----------------------------------------------------------------------------

int topicfs_getattr(const char* path, struct stat* stbuf, struct fuse_file_info* fi)
{
  (void)fi;
  memset(stbuf, 0, sizeof(struct stat));
  std::string spath(path);

  auto* topicfs = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto ros2_node = topicfs->get_ros2_node();

  if (spath == "/")
  {
    stbuf->st_mode = S_IFDIR | 0755;
    stbuf->st_nlink = 2;
    return 0;
  }

  auto topics = ros2_node->get_topics();
  auto services = ros2_node->get_services();
  std::string path_no_slash = spath.substr(1);

  stbuf->st_ino = 1 + std::hash<std::string>{}(spath);

  // Check if path is a directory prefix of any topic or service
  auto is_dir_prefix = [&](const std::vector<std::string>& names) -> bool
  {
    return std::any_of(names.begin(), names.end(), [&](const auto& n) {
      return n.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
             (n.length() == path_no_slash.length() || n[path_no_slash.length()] == '/');
    });
  };

  if (is_dir_prefix(topics) || is_dir_prefix(services))
  {
    stbuf->st_mode = S_IFDIR | 0755;
    stbuf->st_nlink = 2;
    return 0;
  }

  size_t pos = spath.rfind('/');
  if (pos != std::string::npos && pos > 0)
  {
    std::string name = spath.substr(1, pos - 1);
    std::string file = spath.substr(pos + 1);
    std::string original = "/" + name;

    // Topic files
    if (std::find(topics.begin(), topics.end(), name) != topics.end())
    {
      if (file == "latest" || file == "info")
      {
        stbuf->st_mode = S_IFREG | 0444;
        stbuf->st_nlink = 1;
        stbuf->st_mtime = time(nullptr);
        if (file == "latest")
        {
          auto msg = ros2_node->get_latest_message(original);
          if (msg) { stbuf->st_size = msg->size(); }
        }
        else
        {
          auto type = ros2_node->get_topic_type(original);
          std::string type_str = type.value_or("unknown");
          std::string info = "Topic: " + original + "\nType: " + type_str + "\n";
          stbuf->st_size = info.size();
        }
        return 0;
      }
      else if (file == "command" && ros2_node->has_publisher(original))
      {
        stbuf->st_mode = S_IFREG | 0222;
        stbuf->st_nlink = 1;
        stbuf->st_size = 0;
        stbuf->st_mtime = time(nullptr);
        return 0;
      }
    }

    // Service files
    if (std::find(services.begin(), services.end(), name) != services.end())
    {
      if (file == "command")
      {
        stbuf->st_mode = S_IFREG | 0222;
        stbuf->st_nlink = 1;
        stbuf->st_size = 0;
        stbuf->st_mtime = time(nullptr);
        return 0;
      }
      else if (file == "response")
      {
        stbuf->st_mode = S_IFREG | 0444;
        stbuf->st_nlink = 1;
        stbuf->st_mtime = time(nullptr);
        auto resp = ros2_node->get_last_response(original);
        stbuf->st_size = resp ? resp->size() : 0;
        return 0;
      }
    }
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "getattr: path %s not found", spath.c_str());
  return -ENOENT;
}

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
    std::cout << "topicfs_readdir: premature call, returning 0" << std::endl;
    return 0;
  }

  auto ros2_node = topicfs->get_ros2_node();
  if (!ros2_node)
  {
    std::cerr << "topicfs_readdir: ros2_node pointer is null!" << std::endl;
    return -ENOMEM;
  }

  std::string spath(path);
  RCLCPP_INFO(ros2_node->get_logger(), "readdir: path=%s", spath.c_str());

  if (spath == "/")
  {
    if (filler(buf, ".", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "..", nullptr, 0, zero_fill_dir_flags))
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "readdir: filler failed for root directory");
      return -ENOMEM;
    }

    std::set<std::string> top_level_dirs;
    auto topics = ros2_node->get_topics();
    auto services = ros2_node->get_services();
    
    for (const auto& topic : topics)
    {
      size_t pos = topic.find('/');
      //std::string top_level = (pos == std::string::npos) ? topic : topic.substr(0, pos);
      top_level_dirs.insert((pos == std::string::npos) ? topic : topic.substr(0, pos));
    }
    for (const auto& service : services)
    {
      size_t pos = service.find('/');
      top_level_dirs.insert(pos == std::string::npos ? service : service.substr(0, pos));
    }
    for (const auto& dir : top_level_dirs)
    {
      if (filler(buf, dir.c_str(), nullptr, 0, zero_fill_dir_flags))
      {
        RCLCPP_ERROR(ros2_node->get_logger(),
                     "readdir: filler failed for top-level dir %s", dir.c_str());
        return -ENOMEM;
      }
    }
    return 0;
  }

  std::string path_no_slash = spath.substr(1);
  auto topics = ros2_node->get_topics();

  if (std::find(topics.begin(), topics.end(), path_no_slash) != topics.end())
  {
    if (filler(buf, ".", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "..", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "latest", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "info", nullptr, 0, zero_fill_dir_flags))
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "readdir: filler failed for topic directory %s", path_no_slash.c_str());
      return -ENOMEM;
    }
    if (ros2_node->has_publisher("/" + path_no_slash))
    {
      if (filler(buf, "command", nullptr, 0, zero_fill_dir_flags))
      {
        RCLCPP_ERROR(ros2_node->get_logger(),
                     "readdir: filler failed for command file in %s", path_no_slash.c_str());
        return -ENOMEM;
      }
    }
    return 0;
  }

    // Check if path is an exact service
  auto services = ros2_node->get_services();
  if (std::find(services.begin(), services.end(), path_no_slash) != services.end())
  {
    if (filler(buf, ".", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "..", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "command", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "response", nullptr, 0, zero_fill_dir_flags))
    {
      return -ENOMEM;
    }
    return 0;
  }
  
  std::set<std::string> subdirs;
  std::vector<std::string> matching_topics;
  for (const auto& topic : topics)
  {
    if (topic.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
        topic.length() > path_no_slash.length() &&
        topic[path_no_slash.length()] == '/')
    {
      std::string remaining = topic.substr(path_no_slash.length() + 1);
      size_t pos = remaining.find('/');
      std::string next_dir = (pos == std::string::npos) ? remaining : remaining.substr(0, pos);
      subdirs.insert(next_dir);
    }
    else if (topic == path_no_slash)
    {
      matching_topics.push_back(topic);
    }
  }
  
  for (const auto& s : services)
  {
    if (s.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
        s.length() > path_no_slash.length() && s[path_no_slash.length()] == '/')
    {
      std::string rest = s.substr(path_no_slash.length() + 1);
      size_t slash = rest.find('/');
      subdirs.insert(slash == std::string::npos ? rest : rest.substr(0, slash));
    }
  }

  if (!subdirs.empty() || !matching_topics.empty())
  {
    if (filler(buf, ".", nullptr, 0, zero_fill_dir_flags) ||
        filler(buf, "..", nullptr, 0, zero_fill_dir_flags))
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "readdir: filler failed for directory %s", path_no_slash.c_str());
      return -ENOMEM;
    }
    for (const auto& dir : subdirs)
    {
      if (filler(buf, dir.c_str(), nullptr, 0, zero_fill_dir_flags))
      {
        RCLCPP_ERROR(ros2_node->get_logger(),
                     "readdir: filler failed for subdirectory %s", dir.c_str());
        return -ENOMEM;
      }
    }
    return 0;
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "readdir: path %s not found", spath.c_str());
  return -ENOENT;
}

int topicfs_open(const char* path, struct fuse_file_info* fi)
{
  auto* topicfs = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto ros2_node = topicfs->get_ros2_node();

  std::string spath(path);
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    RCLCPP_ERROR(ros2_node->get_logger(), "open: invalid path %s", spath.c_str());
    return -ENOENT;
  }

  std::string topic = spath.substr(1, pos - 1);
  std::string file = spath.substr(pos + 1);
  std::string original_topic = "/" + topic;
  auto topics = ros2_node->get_topics();

  if (std::find(topics.begin(), topics.end(), topic) != topics.end())
  {
    if (file == "latest" || file == "info")
    {
      if ((fi->flags & O_ACCMODE) != O_RDONLY)
      {
        RCLCPP_ERROR(ros2_node->get_logger(),
                     "open: %s/%s is read-only", original_topic.c_str(), file.c_str());
        return -EACCES;
      }
      // Enable direct_io so each read() call hits the FUSE handler rather than
      // the kernel page cache. Required for poll() to observe fresh data.
      fi->direct_io = 1;
      RCLCPP_INFO(ros2_node->get_logger(),
                  "open: opened %s/%s for reading", original_topic.c_str(), file.c_str());
      return 0;
    }
    else if (file == "command" && ros2_node->has_publisher(original_topic))
    {
      if ((fi->flags & O_ACCMODE) != O_WRONLY && (fi->flags & O_ACCMODE) != O_RDWR)
      {
        RCLCPP_ERROR(ros2_node->get_logger(),
                     "open: %s/%s requires write access", original_topic.c_str(), file.c_str());
        return -EACCES;
      }
      RCLCPP_DEBUG(ros2_node->get_logger(),
                   "open: opened %s/%s for writing", original_topic.c_str(), file.c_str());
      return 0;
    }
  }

// Service files
  auto services = ros2_node->get_services();
  if (std::find(services.begin(), services.end(), topic) != services.end())
  {
    if (file == "command")
    {
      if ((fi->flags & O_ACCMODE) != O_WRONLY && (fi->flags & O_ACCMODE) != O_RDWR)
      {
        RCLCPP_ERROR(ros2_node->get_logger(),
                     "open: %s/%s requires write access", original_topic.c_str(), file.c_str());
        return -EACCES;
      }
      RCLCPP_DEBUG(ros2_node->get_logger(),
                   "open: opened %s/%s for writing", original_topic.c_str(), file.c_str());
      return 0;
    }
    else if (file == "response")
    {
      if ((fi->flags & O_ACCMODE) != O_RDONLY)
      {
        RCLCPP_ERROR(ros2_node->get_logger(),
                     "open: %s/%s is read-only", original_topic.c_str(), file.c_str());
        return -EACCES;
      }
      fi->direct_io = 1;
      RCLCPP_DEBUG(ros2_node->get_logger(),
                   "open: opened %s/%s for reading", original_topic.c_str(), file.c_str());
      return 0;
    }
  }
  RCLCPP_ERROR(ros2_node->get_logger(), "open: path %s not found", spath.c_str());
  return -ENOENT;
}

int topicfs_poll(
  const char* path, struct fuse_file_info* fi,
  struct fuse_pollhandle* ph, unsigned* reventsp)
{
     
  (void)fi;
  *reventsp = 0;

  auto* topicfs = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto ros2_node = topicfs->get_ros2_node();

  std::string spath(path);
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    return -ENOENT;
  }

  std::string topic = spath.substr(1, pos - 1);
  std::string file = spath.substr(pos + 1);

  if (file != "latest")
  {
    // Non-pollable files — signal data always available so callers don't block
    *reventsp = POLLIN;
    return 0;
  }

  std::string original_topic = "/" + topic;

  // If we have a message, data is immediately available
  auto msg = ros2_node->get_latest_message(original_topic);
  if (msg)
  {
    *reventsp = POLLIN;
  }

  // Store the poll handle so notify_file_change can wake this waiter.
  // FUSE transfers ownership of ph to us — we are responsible for destroying it.
  if (ph)
  {
    ros2_node->store_poll_handle(original_topic, ph);
  }

  RCLCPP_DEBUG(ros2_node->get_logger(),
               "poll: registered waiter for %s, data_ready=%s",
               original_topic.c_str(), msg ? "yes" : "no");
  return 0;
}

int topicfs_read(
  const char* path, char* buf, size_t size, off_t offset, struct fuse_file_info* fi)
{
  (void)fi;
  auto* topicfs = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto ros2_node = topicfs->get_ros2_node();

  if (offset < 0)
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "read: invalid offset %ld for path %s", offset, path);
    return -EINVAL;
  }

  std::string spath(path);
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    RCLCPP_ERROR(ros2_node->get_logger(), "read: invalid path %s", spath.c_str());
    return -ENOENT;
  }

  std::string topic = spath.substr(1, pos - 1);
  std::string file = spath.substr(pos + 1);
  std::string original_topic = "/" + topic;

  if (file == "latest")
  {
    auto msg = ros2_node->get_latest_message(original_topic);
    if (!msg)
    {
      RCLCPP_DEBUG(ros2_node->get_logger(),
                   "read: no data for %s/latest", original_topic.c_str());
      return -ENOENT;
    }

    const std::string& data = *msg;
    uint64_t version = ros2_node->get_message_version(original_topic);
    RCLCPP_DEBUG(ros2_node->get_logger(),
                 "read: reading %s/latest, version=%lu, data_size=%zu, offset=%ld",
                 original_topic.c_str(), version, data.size(), offset);

    if (static_cast<std::string::size_type>(offset) >= data.size())
    {
      return 0;
    }

    size_t len = std::min(size, data.size() - static_cast<size_t>(offset));
    memcpy(buf, data.c_str() + offset, len);
    RCLCPP_DEBUG(ros2_node->get_logger(),
                 "read: read %zu bytes from %s/latest at offset %ld",
                 len, original_topic.c_str(), offset);
    return len;
  }
  else if (file == "info")
  {
    auto type = ros2_node->get_topic_type(original_topic);
    std::string type_str = type.value_or("unknown");
    std::string info = "Topic: " + original_topic + "\nType: " + type_str + "\n";

    RCLCPP_DEBUG(ros2_node->get_logger(),
                 "read: reading info for %s: %s", original_topic.c_str(), info.c_str());

    if (static_cast<std::string::size_type>(offset) >= info.size())
    {
      return 0;
    }

    size_t len = std::min(size, info.size() - static_cast<std::string::size_type>(offset));
    memcpy(buf, info.c_str() + offset, len);
    return len;
  }
  else if (file == "response")
  {
    auto resp = ros2_node->get_last_response(original_topic);
    if (!resp)
    {
      // No service call made yet
      static const std::string no_response = R"({"status": "no response yet"})";
      if (static_cast<std::string::size_type>(offset) >= no_response.size())
      {
        return 0;
      }
      size_t len = std::min(size, no_response.size() - static_cast<size_t>(offset));
      memcpy(buf, no_response.c_str() + offset, len);
      return len;
    }

    const std::string& data = *resp;
    if (static_cast<std::string::size_type>(offset) >= data.size())
    {
      return 0;
    }
    size_t len = std::min(size, data.size() - static_cast<size_t>(offset));
    memcpy(buf, data.c_str() + offset, len);
    return len;
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "read: path %s not found", spath.c_str());
  return -ENOENT;
}

int topicfs_write(
  const char* path, const char* buf, size_t size, off_t offset, struct fuse_file_info* fi)
{
  (void)fi;
  (void)offset;  // Atomic writes assumed for command files
  auto* topicfs = static_cast<TopicFS*>(fuse_get_context()->private_data);
  auto ros2_node = topicfs->get_ros2_node();

  std::string spath(path);
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    RCLCPP_ERROR(ros2_node->get_logger(), "write: invalid path %s", spath.c_str());
    return -ENOENT;
  }

  std::string name = spath.substr(1, pos - 1);
  std::string file = spath.substr(pos + 1);
  std::string original = "/" + name;

  if (file != "command")
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "write: %s/%s is not writable", original.c_str(), file.c_str());
    return -EROFS;
  }

  std::string data(buf, size);

  // -----------------------------------------------------------------------------
  // Service command — call the service with the JSON request
  // -----------------------------------------------------------------------------
  if (ros2_node->has_service(original))
  {
    RCLCPP_INFO(ros2_node->get_logger(),
                "write: calling service %s with: %s", original.c_str(), data.c_str());
    try
    {
      nlohmann::json j = nlohmann::json::parse(data);
      std::string result = ros2_node->call_service(original, j);
      RCLCPP_INFO(ros2_node->get_logger(),
                  "write: service %s response: %s", original.c_str(), result.c_str());
      return size;
    }
    catch (const nlohmann::json::parse_error& e)
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "write: JSON parse error for service %s: %s", original.c_str(), e.what());
      return -EINVAL;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "write: service call failed for %s: %s", original.c_str(), e.what());
      return -EIO;
    }
  }

  // -----------------------------------------------------------------------------
  // Topic command — publish message to topic
  // -----------------------------------------------------------------------------
  if (!ros2_node->has_publisher(original))
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "write: no publisher or service for %s", original.c_str());
    return -ENOENT;
  }

  RCLCPP_DEBUG(ros2_node->get_logger(),
               "write: writing to %s/command: %s", original.c_str(), data.c_str());

  try
  {
    nlohmann::json j = nlohmann::json::parse(data);

    auto topic_type = ros2_node->get_topic_type(original);
    if (!topic_type)
    {
      RCLCPP_ERROR(ros2_node->get_logger(),
                   "write: unknown type for %s", original.c_str());
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
                   "write: failed to publish message to %s", original.c_str());
      return -EIO;
    }

    RCLCPP_DEBUG(ros2_node->get_logger(),
                 "write: successfully published to %s", original.c_str());
    return size;
  }
  catch (const nlohmann::json::parse_error& e)
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "write: JSON parse error for %s/command: %s", original.c_str(), e.what());
    return -EINVAL;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "write: failed to process %s/command: %s", original.c_str(), e.what());
    return -EIO;
  }
  catch (...)
  {
    RCLCPP_ERROR(ros2_node->get_logger(),
                 "write: unknown error processing %s/command", original.c_str());
    return -EIO;
  }
}

// -----------------------------------------------------------------------------
// Filesystem listing utility
// -----------------------------------------------------------------------------

int collect_entries(
  void* buf, const char* name, const struct stat* stbuf, off_t off, enum fuse_fill_dir_flags flags)
{
  (void)stbuf;
  (void)off;
  (void)flags;
  std::vector<std::string>* entries = static_cast<std::vector<std::string>*>(buf);
  entries->push_back(name);
  return 0;
}

void list_fuse_filesystem(rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Listing FUSE filesystem structure:");
  std::vector<std::string> root_entries;
  if (topicfs_readdir("/", &root_entries, collect_entries, 0, nullptr, zero_readdir_flags) == 0)
  {
    RCLCPP_INFO(logger, "/");
    for (const auto& entry : root_entries)
    {
      if (entry != "." && entry != "..")
      {
        RCLCPP_INFO(logger, "  /%s", entry.c_str());
        std::string t_path = "/" + entry;
        std::vector<std::string> t_entries;
        if (topicfs_readdir(
              t_path.c_str(), &t_entries, collect_entries, 0, nullptr, zero_readdir_flags) == 0)
        {
          for (const auto& topic_entry : t_entries)
          {
            if (topic_entry != "." && topic_entry != "..")
            {
              RCLCPP_INFO(logger, "    /%s/%s", entry.c_str(), topic_entry.c_str());
            }
          }
        }
        else
        {
          RCLCPP_ERROR(logger, "Failed to read topic directory: %s", t_path.c_str());
        }
      }
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to read root directory");
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
