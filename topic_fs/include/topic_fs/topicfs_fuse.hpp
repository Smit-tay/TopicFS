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

#ifndef TOPICFS_FUSE_HPP
#define TOPICFS_FUSE_HPP

// #define FUSE_USE_VERSION 312
#include <fuse3/fuse.h>
#include <fuse3/fuse_common.h>
#include <poll.h>

#include <string>
#include <vector>

#include "topic_fs/topicfs_node.hpp"

int topicfs_getattr(const char* path, struct stat* stbuf, struct fuse_file_info* fi);
int topicfs_readdir(
  const char* path, void* buf, fuse_fill_dir_t filler,
  off_t offset, struct fuse_file_info* fi, enum fuse_readdir_flags flags);
int topicfs_open(const char* path, struct fuse_file_info* fi);
int topicfs_poll(
  const char* path, struct fuse_file_info* fi,
  struct fuse_pollhandle* ph, unsigned* reventsp);
int topicfs_read(const char* path, char* buf, size_t size, off_t offset, struct fuse_file_info* fi);
int topicfs_write(
  const char* path, const char* buf, size_t size, off_t offset, struct fuse_file_info* fi);
void list_fuse_filesystem(rclcpp::Logger logger);
void init_fuse_operations(struct fuse_operations& topicfs_oper);

#endif // TOPICFS_FUSE_HPP
