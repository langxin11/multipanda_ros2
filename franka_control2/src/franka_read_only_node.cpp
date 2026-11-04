// Copyright 2026 multipanda_ros2 contributors
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <franka/robot.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Deliberately independent of Robot's control wrapper: no parameter setters,
// error recovery, stop commands, command subscriptions, or control callbacks.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("franka_read_only");
  const auto robot_ip = node->declare_parameter<std::string>("robot_ip", "");
  int result = 0;
  try {
    if (robot_ip.empty()) {
      throw std::invalid_argument("请指定 robot_ip");
    }
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    franka::Robot robot(robot_ip);
    RCLCPP_INFO(
      node->get_logger(), "只读连接成功：%s，FCI 服务端版本 %u；保留现有机器人参数",
      robot_ip.c_str(), static_cast<unsigned>(robot.serverVersion()));
    sensor_msgs::msg::JointState message;
    message.header.frame_id = "panda_link0";
    for (int i = 1; i <= 7; ++i) {
      message.name.push_back("panda_joint" + std::to_string(i));
    }
    rclcpp::WallRate rate(30.0);
    bool first = true;
    while (rclcpp::ok()) {
      const auto state = robot.readOnce();
      for (std::size_t i = 0; i < state.q.size(); ++i) {
        if (!std::isfinite(state.q[i]) || !std::isfinite(state.dq[i]) ||
          !std::isfinite(state.tau_J[i]))
        {
          throw std::runtime_error("机器人返回非有限关节状态，停止发布");
        }
      }
      if (first) {
        std::ostringstream status;
        status << "mode=" << state.robot_mode << ", errors=" << state.current_errors
               << ", m_ee=" << state.m_ee << ", m_load=" << state.m_load;
        RCLCPP_INFO(node->get_logger(), "真机状态：%s", status.str().c_str());
        first = false;
      }
      if (!rclcpp::ok()) {
        break;
      }
      message.header.stamp = node->now();
      message.position.assign(state.q.begin(), state.q.end());
      message.velocity.assign(state.dq.begin(), state.dq.end());
      message.effort.assign(state.tau_J.begin(), state.tau_J.end());
      publisher->publish(message);
      rclcpp::spin_some(node);
      rate.sleep();
    }
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "只读状态发布失败：%s", error.what());
      result = 1;
    }
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
