// topicfs.hpp
// Copyright 2025 Jack Sidman Smith
//
// Licensed under the MIT License. See LICENSE in project root.

#ifndef TOPICFS_HPP
#define TOPICFS_HPP

#include <memory>
#include <string>
#include <thread>

#include <fuse3/fuse.h>
#include <fuse3/fuse_common.h>

#include "topic_fs/topicfs_node.hpp"

class TopicFS
{
public:
  TopicFS(int argc, char* argv[]);
  ~TopicFS();
  int run();
  std::shared_ptr<topicfsNode> get_ros2_node() const { return ros2_node_; }

private:
  std::shared_ptr<topicfsNode> ros2_node_;
  struct fuse_operations fuse_ops_;
  struct fuse* fuse_handle_;
  std::string mount_point_;
  std::thread ros_thread_;
  unsigned int fuse_threads_{2};

  void cleanup();
  bool initialize_fuse();
  bool initialize_ros(int argc, char* argv[]);
  void run_fuse_loop();
  bool setup_fuse_threads(int argc, char* argv[]);
  bool setup_mount_point(int argc, char* argv[]);
};

#endif // TOPICFS_HPP
