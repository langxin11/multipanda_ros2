// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Adapted for multipanda_ros2: preserve existing robot parameters, Chinese
// prompt, read-only preflight, current-pose equilibrium, and bounded duration.
// Upstream: libfranka 0.9.2 examples/cartesian_impedance_control.cpp
#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <string>
#include <cmath>
#include <functional>
#include <iostream>
#include <thread>
#include <poll.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/control_tools.h>
#include <franka/rate_limiting.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }

// Only a lock-free integer store runs in the robot callback. All terminal I/O
// stays on this ordinary-priority thread, created before robot.control().
class TimeProgress {
 public:
  TimeProgress() : worker_([this] { display(); }) {}
  ~TimeProgress() {
    finished_.store(true, std::memory_order_release);
    worker_.join();
  }
  void update(double seconds) {
    seconds_.store(static_cast<int>(std::clamp(seconds, 0.0, 60.0)),
                   std::memory_order_relaxed);
  }

 private:
  void display() {
    const bool terminal = isatty(STDOUT_FILENO);
    int last = -1;
    const auto render = [&] {
      const int elapsed = seconds_.load(std::memory_order_relaxed);
      if (elapsed == last) {
        return;
      }
      const int filled = elapsed / 3;
      std::cout << (terminal ? "\r" : "")
                << "进度 [" << std::string(filled, '=')
                << std::string(20 - filled, ' ') << "] " << elapsed * 100 / 60
                << "% | 已用 " << elapsed << "/60 秒 | 剩余 " << 60 - elapsed
                << " 秒  " << (terminal ? "" : "\n") << std::flush;
      last = elapsed;
    };
    while (!finished_.load(std::memory_order_acquire)) {
      render();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    render();
    if (terminal) {
      std::cout << std::endl;
    }
  }
  static_assert(std::atomic<int>::is_always_lock_free,
                "Progress updates must be lock-free in the realtime callback");
  std::atomic<int> seconds_{0};
  std::atomic<bool> finished_{false};
  std::thread worker_;
};
}  // namespace

// This controller constrains end-effector pose, not all seven joint positions.
// Like the upstream example, it does not implement nullspace posture control.
int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "用法：cartesian_impedance_demo <机器人IP> [--check]\n"
              << "--check 仅检查状态；默认等待现场按回车，然后体验最多 60 秒。\n";
    return 0;
  }
  if (argc < 2 || argc > 3 || (argc == 3 && std::string(argv[2]) != "--check")) {
    std::cerr << "用法：cartesian_impedance_demo <机器人IP> [--check]\n";
    return 2;
  }
  const bool check_only = argc == 3;
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::Matrix<double, 6, 6> stiffness, damping;
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    auto preflight = robot.readOnce();
    std::cout << "FCI 服务端版本：" << robot.serverVersion()
              << "，模式：" << preflight.robot_mode
              << "，当前错误：" << preflight.current_errors << '\n'
              << "末端质量：" << preflight.m_ee << " kg，附加载荷："
              << preflight.m_load << " kg\n";
    if (check_only) {
      std::cout << "只读检查完成，未启用控制，未写入参数。\n";
      return 0;
    }
    if (!franka::hasRealtimeKernel()) {
      std::cerr << "此体验入口要求实时内核。\n";
      return 1;
    }
    std::cout << "笛卡尔阻抗体验：平移刚度 150 N/m，旋转刚度 10 Nm/rad。\n"
              << "保留现有碰撞阈值、阻抗及负载配置；不调用自动错误恢复。\n"
              << "确认周围无障碍、停止装置可立即操作。不要握住关节或进入夹挤区域。\n"
              << "以控制开始时的末端位姿为平衡点；轻推会产生回复力。\n"
              << "最多运行 60 秒；Ctrl+C 结束软件控制，异常时使用物理停止装置。\n"
              << "请在机器人旁按回车开始；Ctrl+C 取消：" << std::flush;
    std::string confirmation;
    pollfd input{STDIN_FILENO, POLLIN, 0};
    bool confirmed = false;
    while (!stop_requested) {
      const int ready = poll(&input, 1, 100);
      if (ready > 0) {
        confirmed = static_cast<bool>(std::getline(std::cin, confirmation));
        break;
      }
      if (ready < 0) {
        break;
      }
    }
    if (!confirmed || stop_requested) {
      std::cout << "\n已取消，未启用控制。\n";
      return 0;
    }
    preflight = robot.readOnce();
    if (preflight.robot_mode != franka::RobotMode::kIdle || preflight.current_errors ||
        std::any_of(preflight.dq.begin(), preflight.dq.end(),
                    [](double dq) { return !std::isfinite(dq) || std::abs(dq) > 0.02; })) {
      std::cerr << "机器人必须无错误、处于 Idle 且静止；本次未启动控制。\n";
      return 1;
    }
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;
    bool first_cycle = true;
    double elapsed = 0.0;
    std::cout << "已开始阻抗控制；Ctrl+C 结束，60 秒后自动结束。" << std::endl;
    {
      TimeProgress progress;
      // define callback for the torque control loop
      std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
          impedance_control_callback = [&](const franka::RobotState& robot_state,
                                           franka::Duration duration) -> franka::Torques {
        elapsed += duration.toSec();
        progress.update(elapsed);
        if (first_cycle) {
          const Eigen::Affine3d initial(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          position_d = initial.translation();
          orientation_d = Eigen::Quaterniond(initial.rotation());
          first_cycle = false;
        }
        // get state variables
        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        std::array<double, 42> jacobian_array =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.rotation());

        // compute error to desired equilibrium pose
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d;

        // orientation error
        // "difference" quaternion
        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
          orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.rotation() * error.tail(3);

        // compute control
        Eigen::Matrix<double, 7, 1> tau_task, tau_d;

        // Spring damper system with damping ratio=1
        tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
        tau_d << tau_task + coriolis;

        std::array<double, 7> tau_d_array{};
        Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;
        const auto limited = franka::limitRate(franka::kMaxTorqueRate, tau_d_array,
                                               robot_state.tau_J_d);
        franka::Torques command(limited);
        if (stop_requested || elapsed >= 60.0) {
          return franka::MotionFinished(command);
        }
        return command;
      };

      robot.control(impedance_control_callback, true);
    }  // Join the display thread before the final message or an exception report.
    std::cout << "阻抗体验已结束。" << std::endl;
  } catch (const franka::Exception& ex) {
    // print exception
    std::cerr << "控制未完成：" << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
