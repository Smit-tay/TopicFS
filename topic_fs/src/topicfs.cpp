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
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

// Project
#include "topic_fs/topicfs.hpp"
#include "topic_fs/topicfs_fuse.hpp"

// -----------------------------------------------------------------------------
// Signal handling
// -----------------------------------------------------------------------------

static struct fuse* g_fuse_handle = nullptr;

static void signal_handler(int signum)
{
  (void)signum;
  if (g_fuse_handle)
  {
    fuse_exit(g_fuse_handle);
  }
}

// -----------------------------------------------------------------------------
// Constructor / Destructor
// -----------------------------------------------------------------------------

TopicFS::TopicFS(int argc, char* argv[]) : fuse_handle_(nullptr)
{
  if (!initialize_ros(argc, argv))
  {
    throw std::runtime_error("ROS2 initialization failed");
  }
  if (!setup_mount_point(argc, argv))
  {
    throw std::runtime_error("Mount point setup failed");
  }
  if (!setup_fuse_threads(argc, argv))
  {
    throw std::runtime_error("FUSE thread count setup failed");
  }
  init_fuse_operations(fuse_ops_);
}

TopicFS::~TopicFS()
{
  cleanup();
}

// -----------------------------------------------------------------------------
// ROS2 initialisation
// -----------------------------------------------------------------------------

bool TopicFS::initialize_ros(int argc, char* argv[])
{
  try
  {
    rclcpp::init(argc, argv);
    ros2_node_ = std::make_shared<topicfsNode>();
    RCLCPP_INFO(ros2_node_->get_logger(), "Topic FS Utility - Created by Jack Sidman Smith");
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "ROS2 initialization failed: " << e.what() << std::endl;
    return false;
  }
}

// -----------------------------------------------------------------------------
// Mount point setup
// -----------------------------------------------------------------------------

bool TopicFS::setup_mount_point(int argc, char* argv[])
{
  ros2_node_->declare_parameter<std::string>("mount_point", "~/fuse_mount");
  ros2_node_->get_parameter("mount_point", mount_point_);

  for (int i = 1; i < argc; ++i)
  {
    std::string arg(argv[i]);
    if (arg.find("mount_point:=") == 0)
    {
      mount_point_ = arg.substr(std::string("mount_point:=").length());
      RCLCPP_INFO(ros2_node_->get_logger(), "Overriding mount_point: %s", mount_point_.c_str());
    }
  }

  if (!mount_point_.empty() && mount_point_[0] == '~')
  {
    const char* home = getenv("HOME");
    if (!home)
    {
      RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to resolve ~: HOME not set");
      return false;
    }
    mount_point_ = std::string(home) + mount_point_.substr(1);
  }

  FILE* mount_file = fopen("/proc/mounts", "r");
  if (mount_file)
  {
    char line[1024];
    while (fgets(line, sizeof(line), mount_file))
    {
      if (strstr(line, mount_point_.c_str()) && strstr(line, "fuse"))
      {
        RCLCPP_WARN(ros2_node_->get_logger(), "Mount point %s already in use",
                    mount_point_.c_str());
        std::string cmd = "fusermount3 -u " + mount_point_;
        if (system(cmd.c_str()) != 0)
        {
          RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to unmount %s: %s",
                       mount_point_.c_str(), strerror(errno));
          fclose(mount_file);
          return false;
        }
        RCLCPP_INFO(ros2_node_->get_logger(), "Unmounted stale mount point %s",
                    mount_point_.c_str());
        break;
      }
    }
    fclose(mount_file);
  }

  struct stat st;
  if (stat(mount_point_.c_str(), &st) != 0)
  {
    if (mkdir(mount_point_.c_str(), 0755) != 0)
    {
      RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to create mount point %s: %s",
                   mount_point_.c_str(), strerror(errno));
      return false;
    }
  }
  else if (!S_ISDIR(st.st_mode))
  {
    RCLCPP_ERROR(ros2_node_->get_logger(), "Mount point %s is not a directory",
                 mount_point_.c_str());
    return false;
  }
  else if (access(mount_point_.c_str(), W_OK) != 0)
  {
    RCLCPP_ERROR(ros2_node_->get_logger(), "Mount point %s is not writable: %s",
                 mount_point_.c_str(), strerror(errno));
    return false;
  }

  ros2_node_->set_mount_point(mount_point_);
  return true;
}

// -----------------------------------------------------------------------------
// FUSE thread count setup
// -----------------------------------------------------------------------------

bool TopicFS::setup_fuse_threads(int argc, char* argv[])
{
  // Default: half the logical CPUs, minimum 2
  unsigned int hw = std::thread::hardware_concurrency();
  unsigned int default_threads = std::max(2u, hw / 2u);

  ros2_node_->declare_parameter<int>("fuse_threads", static_cast<int>(default_threads));

  int param_threads = static_cast<int>(default_threads);
  ros2_node_->get_parameter("fuse_threads", param_threads);

  for (int i = 1; i < argc; ++i)
  {
    std::string arg(argv[i]);
    if (arg.find("fuse_threads:=") == 0)
    {
      try
      {
        param_threads = std::stoi(arg.substr(std::string("fuse_threads:=").length()));
      }
      catch (const std::exception&)
      {
        RCLCPP_ERROR(ros2_node_->get_logger(),
                     "fuse_threads: invalid value '%s', must be an integer >= 2",
                     arg.c_str());
        return false;
      }
    }
  }

  if (param_threads < 2)
  {
    RCLCPP_ERROR(ros2_node_->get_logger(),
                 "fuse_threads: value %d is below minimum (2). "
                 "At least 2 threads are required for poll() support.",
                 param_threads);
    return false;
  }

  fuse_threads_ = static_cast<unsigned int>(param_threads);
  RCLCPP_INFO(ros2_node_->get_logger(),
              "FUSE thread count: %u (hw_concurrency=%u, default=%u)",
              fuse_threads_, hw, default_threads);
  return true;
}

// -----------------------------------------------------------------------------
// FUSE initialisation
// -----------------------------------------------------------------------------

bool TopicFS::initialize_fuse()
{
  list_fuse_filesystem(ros2_node_->get_logger());

  std::vector<char*> fuse_args = {
    (char*)"topic_fs",
    (char*)"-o",
    (char*)"entry_timeout=0,attr_timeout=0",
    nullptr
  };
  struct fuse_args args = FUSE_ARGS_INIT(static_cast<int>(fuse_args.size() - 1),
                                          fuse_args.data());

  fuse_handle_ = fuse_new(&args, &fuse_ops_, sizeof(fuse_ops_), this);
  if (!fuse_handle_)
  {
    RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to initialize FUSE: %s", strerror(errno));
    return false;
  }

  // Register global handle for signal handler
  g_fuse_handle = fuse_handle_;

  ros2_node_->set_fuse_handle(fuse_handle_);

  if (fuse_mount(fuse_handle_, mount_point_.c_str()) != 0)
  {
    RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to mount FUSE at %s: %s",
                 mount_point_.c_str(), strerror(errno));
    fuse_destroy(fuse_handle_);
    fuse_handle_ = nullptr;
    g_fuse_handle = nullptr;
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
// FUSE loop
// -----------------------------------------------------------------------------

void TopicFS::run_fuse_loop()
{
  // Install signal handler to cleanly exit FUSE loop on SIGINT/SIGTERM
  struct sigaction sa;
  sa.sa_handler = signal_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  RCLCPP_INFO(ros2_node_->get_logger(), 
              "Running FUSE filesystem at %s (threads=%u)",
              mount_point_.c_str(), fuse_threads_);

  struct fuse_loop_config* loop_config = fuse_loop_cfg_create();
fuse_loop_cfg_set_max_threads(loop_config, fuse_threads_ * 2);
fuse_loop_cfg_set_idle_threads(loop_config, static_cast<int>(fuse_threads_));

int ret = fuse_loop_mt(fuse_handle_, loop_config);
fuse_loop_cfg_destroy(loop_config);

if (ret != 0 && ret != -EINTR)
  {
    RCLCPP_ERROR(ros2_node_->get_logger(), 
                 "FUSE loop exited with error: %d (%s)", 
                 ret, strerror(-ret));
  }

  // Restore default signal handling for clean ROS2 shutdown
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);
}

// -----------------------------------------------------------------------------
// Cleanup
// -----------------------------------------------------------------------------

void TopicFS::cleanup()
{
  // First stop the ROS2 thread
  if (ros_thread_.joinable())
  {
    rclcpp::shutdown();
    ros_thread_.join();
  }

  // Then cleanup FUSE
  if (fuse_handle_)
  {
    fuse_unmount(fuse_handle_);
    fuse_destroy(fuse_handle_);
    fuse_handle_ = nullptr;
    g_fuse_handle = nullptr;
  }
}

// -----------------------------------------------------------------------------
// Run
// -----------------------------------------------------------------------------

int TopicFS::run()
{
  if (!initialize_fuse())
  {
    cleanup();
    return 1;
  }

  ros_thread_ = std::thread([this]()
  {
    rclcpp::spin(ros2_node_);
    rclcpp::shutdown();
  });

  run_fuse_loop();
  cleanup();
  return 0;
}
