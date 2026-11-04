# Jazzy 适配验证记录

验证日期：2026-11-04。分支：`jazzy`，基于原 `humble` 分支的 `44076f1`。

## 环境

本次在 x86_64 Docker 容器中实际编译、启动节点并发送运动指令。

| 组件 | 实测版本 |
|---|---|
| 容器系统 | Ubuntu 24.04.5 LTS |
| ROS | Jazzy |
| GCC | 13.3.0 |
| Eigen | 3.4.0 |
| ros2_control / controller_manager | 4.48.1 |
| MoveIt | 2.12.4 |
| libfranka | 0.9.2，源码编译 |
| MuJoCo | 3.2.0，官方 Linux 包，SHA-256 校验通过 |
| DQ Robotics | `d4fd2838fe26cccaa4b6e45ad9c7e5e628d10213`，源码编译 |

## 验证结果

| 检查项 | 结果与范围 |
|---|---|
| 工作区编译 | 28 个 ROS 包均通过；含真机接口、仿真、多模式控制、运动生成器及 Garmi 包 |
| URDF | 6 个 pytest 用例通过；colcon 汇总计 7 项，0 错误、0 失败 |
| 单臂 mock | 7 个关节反馈、关节广播器激活通过 |
| 双臂 mock | 14 个关节反馈、关节广播器激活通过 |
| 单臂 MuJoCo | 笛卡尔阻抗切换到关节阻抗成功；关节 1 响应指令；夹爪 Move 动作成功 |
| 双臂 MuJoCo | 耦合笛卡尔控制激活成功；切换到两个独立关节阻抗控制后，两臂均响应指令 |
| 单臂 MoveIt | OMPL 规划、时间参数化与轨迹执行通过；16 个轨迹点，关节 1 目标误差约 0.00043 rad |
| 双臂 MoveIt | 14 关节联合规划与轨迹执行通过；16 个轨迹点，左臂关节 1 目标误差约 0.00025 rad |
| 图形窗口 | Xvfb + Mesa 软件渲染下启动 MuJoCo viewer 和 RViz，并通过单臂运动与夹爪检查 |

原始机器可读结果：[无界面验证](validation/jazzy-headless-results.json)、[窗口验证](validation/jazzy-gui-results.json)、[双臂 MoveIt](validation/jazzy-dual-moveit-results.json)。
运动位移是测试观察到达到阈值时的值，不是稳态精度评估。测试是短时集成检查，不代表长期稳定性或控制性能认证。

## 关闭状态与已知问题

**初轮功能检查通过，但有子节点关闭异常；最新提交前回归的六个场景均功能通过且正常关闭。**

- 已修复本项目 MuJoCo 控制器资源释放顺序、`RobotSim` 缺失析构函数，以及仿真模型数组释放方式；MuJoCo 服务器能正常退出。
- 实测 MoveIt 2.12.4 在关闭时发生 `TrajectoryExecutionManager` / `CallbackGroup` 析构崩溃。本分支只在 `move_group` 进程预加载 Simple Controller Manager 插件，让插件代码保留至进程退出；应用后规划、执行和 MoveIt 退出均通过。上游有相似堆栈的报告：[MoveIt #1597](https://github.com/moveit/moveit2/issues/1597)、[MoveIt #3721](https://github.com/moveit/moveit2/issues/3721)。相似报告不能单独证明相同根因，此处以本机复现与回归结果为依据。
- 发行版提供的 `joint_state_publisher`、`rosapi_node` 在接收关闭信号时仍可能出现 rclpy 异常并以 1 退出。本次记录中有复现，发生在功能测试结束后的关闭阶段；未修改系统 ROS 包来隐藏这些异常。
- `results.json` 中 `passed` 表示运行功能检查，`shutdown_clean` 表示关闭检查。开启 `--strict-shutdown` 会将关闭异常计入脚本失败；初轮记录未满足该要求，最新一轮全部为 true，但不据此声称偶发关闭问题已消除。

## 复现

安装和构建步骤见 [中文使用说明](../README.zh-CN.md)。容器内执行：

```bash
source install/setup.bash
colcon test --packages-select franka_description --ctest-args -R urdf_tests
colcon test-result --verbose
python3 src/tools/verify_jazzy.py
python3 src/tools/verify_jazzy.py --strict-shutdown
```

窗口检查可在有 DISPLAY 的终端加 `--gui`；无显示器环境：

```bash
xvfb-run -a -s '-screen 0 1600x1200x24' env LIBGL_ALWAYS_SOFTWARE=1 \
  python3 src/tools/verify_jazzy.py --scenario single --gui \
  --output log/jazzy-gui-verification
```

完整日志默认写入工作区的 `log/jazzy-verification/`。本次本地日志另保存在仓库的 `.jazzy_validation/`，不纳入 Git；关键 JSON 结果随本记录保存。

## 尚未验证

- 真实 Panda 的轨迹运动执行、长期实时性能、错误恢复、碰撞行为及负载配置修改。下述初步姿态保持验证不覆盖这些功能。
- 真机与仿真混合的双臂、四臂场景，以及 Garmi 的运行功能。
- 相机、激光雷达、所有控制模式及全部运动生成器的组合。
- MoveIt 数据库存储：原有 `db:=true` 仍引用旧 MongoDB 启动项，默认关闭；安装 SQLite 包并不等于数据库功能已迁移。
- 宿主机原生安装、MuJoCo 在物理显卡上的完整运行及全部桌面交互。最初的仿真窗口测试使用容器内软件渲染；真机只读 RViz 的实际桌面显示已补充验证，见下文。
- GitHub Actions 已配置 Jazzy 编译与功能测试，但本地分支尚未推送，未获得远端 CI 运行结果。

libfranka 版本仍需匹配你的 Panda FCI；仿真验证不能代替真机验证。

## 真机只读补充验证（2026-11-04）

设备为未安装夹爪的 Panda；通过设备自身 `/admin/api/system-version` GET 接口读取到
系统版本 4.2.2，libfranka 0.9.2 实际握手得到 FCI 服务端版本 5。
宿主机内核为 `6.8.1-1045-realtime`，`/sys/kernel/realtime` 为 1。
设备读回模式为 Idle，当前错误为空，`m_ee` 与 `m_load` 均为 0 kg。

新增 `franka_read_only.launch.py` 使用独立的 `franka_read_only_node`，仅调用
libfranka 构造连接和 `readOnce()`，不使用会写入默认参数的硬件控制封装。
在 `multipanda-jazzy` 容器、ROS_DOMAIN_ID=172 下实测：

- 修改涉及的 `franka_control2` 和 `franka_bringup` 均构建通过。
- 7 秒订阅窗口收到 209 条消息，约 30.00 Hz；七个关节的位置、速度和力矩均为有限数。
- 最后一帧消息年龄约 0.00073 秒；`panda_link0` 到 `panda_link8` 的 TF 可解析。
- `/joint_states` 唯一发布者为只读节点；该 ROS 域没有 controller_manager 或 move_group。
- RViz 在用户桌面的 DISPLAY=:0 启动，日志报告 OpenGL 4.6；窗口截图确认无夹爪模型正常显示，Global Status 和 RobotModel Status 均为 OK。
- 向只读 launch 发送 SIGINT 后，读状态节点、robot_state_publisher、RViz 均正常退出。

机器可读结果：[真机只读验证](validation/jazzy-panda-read-only-results.json)。
该结果仅确认真机状态链路，不包含运动控制或参数写入；自动仿真脚本不会启动此入口。

## 真机 MoveIt 准备与姿态保持（2026-11-04）

在用户确认现场空间与停止装置后，进一步验证单臂无夹爪 MoveIt：

- 修正无夹爪 SRDF 不应包含 `panda_hand_tcp` 链；移除无夹爪配置中的夹爪执行接口。
- 真机 MoveIt 默认保留现有机器人阻抗/碰撞参数；不加载夹爪，轨迹控制器默认 inactive。
- 使用真实当前状态，OMPL 成功规划第七关节 +0.02 rad 的目标，生成 14 个轨迹点，时长约 1.27 秒；该轨迹未执行。
- 激活 `panda_arm_controller`，进入 joint_torque 姿态保持；观察 5 秒，七关节相对激活前的最大偏移约 0.000416 rad（0.024°）。
- 真机硬件插件、joint_state_broadcaster、JointTrajectoryController 和 MoveIt 的连接已运行验证。
- 独立真机 RViz 配置使用 `panda_arm` 和 5% 速度/加速度缩放。

机器可读结果：[真机 MoveIt 准备验证](validation/jazzy-panda-moveit-results.json)。
这只证明短时激活及保持和规划链路通过，不表示真实轨迹跟踪或全部控制模式已验证。

## 笛卡尔阻抗体验（用户现场验证，2026-11-04）

`cartesian_impedance_demo` 基于 libfranka 0.9.2 官方示例改编，保留现有机器人参数，
使用平移刚度 150 N/m、旋转刚度 10 Nm/rad，并以控制开始时的末端位姿为平衡点。
新增中文确认、60 秒限时、Ctrl+C 结束及独立线程的时间进度显示。

- 开发环境中编译通过；实际只读连接返回 Idle、无错误、末端和附加载荷均为 0 kg。
- 启动前 Ctrl+C 取消测试通过，没有开启控制。
- 无硬件显示预览覆盖 0%、25%、100%、提前结束和异常清理；实时回调只写入无锁原子计数，不执行终端输出。
- 用户现场体验后提供完整终端输出，显示运行到 `100% | 已用 60/60 秒 | 剩余 0 秒`，随后输出“阻抗体验已结束”，没有控制异常信息。

机器可读记录：[用户提供的阻抗体验结果](validation/jazzy-panda-impedance-results.json)。
这项真机证据来自用户提供的终端日志及体验确认，并非自动化运动测试。
没有收集高频力矩/位姿日志，不据此宣称长期稳定性、轨迹精度或碰撞保护性能通过验证。

## 提交前回归（2026-11-04）

在独立验证容器中针对最终代码重新检查，没有连接真机：

- 全工作区 28 个包构建通过（约 1 分 15 秒）；保留上游弃用 API 和未使用参数警告。
- URDF 测试汇总 7 项，0 错误、0 失败、0 跳过。
- mock 单/双臂、MuJoCo 单/双臂、MoveIt 单/双臂六个场景全部通过，且本轮 `shutdown_clean` 均为 true。
- 从固定提交导出干净的 MuJoCo 子模块，应用补丁后与当前全部六个修改/新增文件逐字节一致，反向补丁检查也通过。

完整机器可读结果：[提交前回归](validation/jazzy-precommit-results.json)。
