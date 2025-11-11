#ifndef TOPICFS_HPP
#define TOPICFS_HPP
#include "topicfs_node.hpp"

// #define FUSE_USE_VERSION 31  // required here because of member: struct fuse* fuse_handle_;
#include <fuse3/fuse.h>
#include <memory>
#include <string>
#include <thread>

class TopicFS {
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
  bool initialize_ros(int argc, char* argv[]);
  bool setup_mount_point(int argc, char* argv[]);
  bool initialize_fuse();
  void run_fuse_loop();
  void cleanup();
};

#endif // TOPICFS_HPP