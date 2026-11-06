# 多模式运动生成器（ROS 2 Jazzy）

这些 C++ 例程通过 ROS action 生成关节或笛卡尔运动，配合
[多模式控制器](../../docs/chapters/franka_multi_mode_controller.md)使用。
环境和完整构建步骤见[中文指南](../../README.zh-CN.md)。
下列命令在容器工作区 `~/multipanda_ws` 中运行，每个终端先执行
`source install/setup.bash`，所有终端使用相同 ROS_DOMAIN_ID。

## 构建与接口

```bash
./src/tools/build_jazzy --packages-select panda_motion_generator_msgs panda_motion_generators
source install/setup.bash
ros2 pkg executables panda_motion_generators
```

新增消息或 action 时，更新 `panda_motion_generator_msgs/CMakeLists.txt`；
新增节点时，更新本包 [CMakeLists.txt](CMakeLists.txt) 的构建和安装条目。

两个生成器节点均需要五个位置参数，按以下顺序传入：

1. action 名称；
2. 获取机器人状态的服务名；
3. 多模式控制器节点名称；
4. controllet 名称；
5. 目标发布话题。

ROS 参数放在位置参数之后的 `--ros-args` 中。缺少位置参数时节点输出用法并退出，
不会自动选择机器人或发送目标。以前教程中的 `mm_go`、`comtest` 是作者本地别名，
不是本仓库安装的命令。

## 单臂 MuJoCo 中启动关节生成器

终端一启动仿真：

```bash
ros2 launch franka_bringup franka_sim.launch.py unpause:=true
```

终端二停用默认笛卡尔阻抗控制器，再加载多模式控制器：

```bash
ros2 control set_controller_state cartesian_impedance_controller inactive
ros2 run controller_manager spawner multi_mode_controller \
  --param-file "$(ros2 pkg prefix franka_bringup)/share/franka_bringup/config/sim/single_sim_controllers.yaml"
ros2 control list_controllers
ros2 service list -t
```

确认 `multi_mode_controller` 为 active，且 `/panda/get_robot_states` 可用后启动：

```bash
ros2 run panda_motion_generators panda_poly_c2_joint_motion_generator_node \
  joint_via_motion /panda/get_robot_states multi_mode_controller \
  panda_joint_impedance_controller /panda/panda_joint_impedance_controller/desired_pose \
  --ros-args -p use_sim_time:=true
```

终端三检查 action 接口及示例客户端源码：

```bash
ros2 action list -t
ros2 interface show panda_motion_generator_msgs/action/JointViaMotion
```

[panda_to_init_pose_client.cpp](src/panda_to_init_pose_client.cpp) 和
[panda_poly_c2_joint_motion_generator_client.cpp](src/panda_poly_c2_joint_motion_generator_client.cpp)
包含预设运动目标；阅读并确认目标后才运行相应客户端。生成器节点本身只等待请求。
原先用 `communication_test` 充当“复位”命令的步骤已移除：该程序会驱动机器人运动，
且本项目 Docker 镜像没有构建 libfranka 的全部上游示例。

## 笛卡尔生成器与真机

笛卡尔节点可使用以下对应参数；与关节生成器分别运行，避免同时发送目标：

```bash
ros2 run panda_motion_generators panda_dq_c1_cartesian_motion_generator_node \
  cartesian_via_motion /panda/get_robot_states multi_mode_controller \
  panda_cartesian_impedance_controller /panda/panda_cartesian_impedance_controller/desired_pose \
  --ros-args -p use_sim_time:=true
```

真机多模式入口是 `franka_bringup/multimode_franka.launch.py`，使用
`robot_ip_1`、`arm_id_1`、`load_gripper_1` 参数；不能沿用仿真时间配置。
该入口会激活控制器，操作前按[真机指南](../../README.zh-CN.md#真机使用)准备。
这些生成器的所有运动组合尚未完成 Jazzy 运行验证，现有证据范围见
[验证记录](../../docs/jazzy-validation.zh-CN.md)。
