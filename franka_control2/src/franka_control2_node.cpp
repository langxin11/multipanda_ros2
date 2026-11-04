// Copyright 2020 ROS2-Control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Keep the existing executable name while using Jazzy's maintained control loop.
// execv preserves ROS arguments, signals and the child process exit status.
#include <ament_index_cpp/get_package_prefix.hpp>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <unistd.h>

int main(int, char ** argv)
{
  const auto executable = ament_index_cpp::get_package_prefix("controller_manager") +
    "/lib/controller_manager/ros2_control_node";
  execv(executable.c_str(), argv);
  std::cerr << "无法启动 Jazzy controller_manager: " << std::strerror(errno) << std::endl;
  return 127;
}
