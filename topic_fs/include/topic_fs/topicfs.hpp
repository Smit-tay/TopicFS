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
