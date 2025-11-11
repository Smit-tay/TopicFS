#ifndef TOPICFS_FUSE_HPP
#define TOPICFS_FUSE_HPP

// #define FUSE_USE_VERSION 31  // TODO(Jack): Update to latest FUSE version
#include <fuse3/fuse.h>
#include <fuse3/fuse_common.h>
#include <string>
#include <vector>
#include "topic_fs/topicfs_node.hpp"

//extern std::shared_ptr<topicfsNode> ros2_node;
//extern struct fuse_operations topicfs_oper;

int topicfs_getattr(const char* path, struct stat* stbuf, struct fuse_file_info* fi);
int topicfs_readdir(const char* path, void* buf, fuse_fill_dir_t filler, off_t offset, struct fuse_file_info* fi);
int topicfs_open(const char* path, struct fuse_file_info* fi);
int topicfs_read(const char* path, char* buf, size_t size, off_t offset, struct fuse_file_info* fi);
int topicfs_write(const char* path, const char* buf, size_t size, off_t offset, struct fuse_file_info* fi);
void list_fuse_filesystem(rclcpp::Logger logger);
void init_fuse_operations(struct fuse_operations& topicfs_oper);

#endif // TOPICFS_FUSE_HPP
