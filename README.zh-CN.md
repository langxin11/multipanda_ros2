# multipanda_ros2：Ubuntu 24.04 / ROS 2 Jazzy

本分支面向 Franka Panda，提供单臂、双臂的 ros2_control 接口与 MuJoCo 仿真。
开发脚本采用中文提示。依赖安装在 Docker 镜像内，源码在宿主机编辑，构建结果保存在 Docker 工作区卷中。

## 快速开始

在本仓库的 `jazzy` 分支根目录执行：

```bash
./tools/setup_env
./run
```

首次构建需要从 Ubuntu、ROS 和 GitHub 下载依赖。脚本会传递终端中的 HTTP/HTTPS 代理变量；检测到 `127.0.0.1` 或 `localhost` 代理时，构建阶段使用宿主机网络。可用 `MULTIPANDA_BUILD_NETWORK` 覆盖这一选择。默认同时编译 4 个任务；内存较少时可以使用：

```bash
./tools/setup_env --build-arg BUILD_JOBS=2
```

进入容器后：

```bash
./src/tools/build_jazzy
source install/setup.bash
ros2 launch franka_bringup franka_sim.launch.py unpause:=true
```

双臂仿真：

```bash
ros2 launch franka_bringup dual_franka_sim.launch.py unpause:=true
```

打开另一个容器终端：

```bash
docker exec -it --user developer multipanda-jazzy bash
```

容器启动后会自动加载 Jazzy 及已构建的工作区。修改 C++ 后重新运行构建脚本，并在启动节点前重新 `source install/setup.bash`。

## 无界面运行与自动验证

SSH 或不需要显示窗口时，在宿主机运行：

```bash
./run --headless
```

容器内可运行：

```bash
source install/setup.bash
ros2 launch franka_bringup franka_sim.launch.py unpause:=true headless:=true no_render:=true
```

自动验证使用独立的 ROS_DOMAIN_ID 和本机发现范围，只启动 mock 硬件或 MuJoCo：

```bash
python3 src/tools/verify_jazzy.py
```

脚本检查单臂与双臂 mock 硬件启动、单臂控制器切换与关节指令响应、夹爪动作、双臂多模式切换与两臂运动，以及单臂/双臂 MoveIt 规划和轨迹执行。
报告与完整启动日志位于 `log/jazzy-verification/`。`passed` 表示功能检查结果，`shutdown_clean` 单独记录关闭状态；加上 `--strict-shutdown` 后，子节点关闭异常也会使脚本返回失败。当前发行版自带的部分 Python 节点仍可能在关闭时报告异常，详见验证记录。单独运行一个场景：

```bash
python3 src/tools/verify_jazzy.py --scenario single
```

URDF 测试：

```bash
colcon test --packages-select franka_description --ctest-args -R urdf_tests
colcon test-result --verbose
```

实际验证范围及尚未验证的功能见 [Jazzy 验证记录](docs/jazzy-validation.zh-CN.md)。

## 依赖与版本

| 组件 | 本分支配置 |
|---|---|
| 系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| Eigen | Ubuntu 系统版本 3.4.0 |
| 编译器 | Ubuntu 默认 GCC |
| libfranka | 0.9.2，面向匹配该版本 FCI 的 Panda |
| MuJoCo | 3.2.0 |
| DQ Robotics | 固定源码提交，见 `tools/Dockerfile` |

本分支使用系统 Eigen，无需卸载或另装旧版。Python 常用依赖通过 apt 安装；
额外 Python 包应放在可访问系统 ROS 包的虚拟环境中，不向系统 Python 全局执行 `pip install`。

进一步阅读：[模块文档](docs/main.md)、[自定义控制器教程](franka_example_controllers/README.md)、
[多模式运动生成器说明](franka_multi_mode_controller/panda_motion_generators/Felix_README.md)。
教程中的 `franka_controllers/JointImpedanceController` 等名称是保留的插件别名，
实际 ROS 包名仍是 `franka_example_controllers`。

Docker 与原生安装使用同一份源码。如果已有原生 Jazzy 工作区，需要先安装与
[Dockerfile](tools/Dockerfile) 一致的系统依赖、libfranka、MuJoCo 和 DQ Robotics，再运行：

```bash
./tools/prepare_jazzy
./tools/build_jazzy
source .jazzy_ws/install/setup.bash
```

仓库直接位于工作区的 `src` 目录时，构建脚本使用其父目录作为工作区；否则默认使用仓库内的 `.jazzy_ws`。
可以通过 `MULTIPANDA_WS` 指定工作区路径。

## MuJoCo 子模块补丁

上游子模块仍基于 Humble。本分支保留固定提交，并将 Jazzy 差异保存在
`tools/patches/mujoco_ros_pkgs-jazzy.patch`，由 `tools/prepare_jazzy` 幂等应用。
因此初始化后，`git status` 显示 MuJoCo 子模块有修改是预期行为。

这个补丁随主仓库一起保存；不依赖未发布的子模块提交。更新或重置子模块后需要重新运行准备脚本。
如果补丁与自己的修改冲突，脚本会停止并保留现有修改。

## 控制器与启动配置的变化

- Jazzy 控制器通过 spawner 的 `--param-file` 显式接收参数文件。
- 真机启动从 `robot_state_publisher` 的 `robot_description` 话题获取 URDF。
- `franka_control2_node` 保留原可执行文件名，转交给 Jazzy 自带的 `ros2_control_node`，使用发行版维护的控制循环。
- mock 硬件使用 `mock_components/GenericSystem`，旧 launch 参数 `use_fake_hardware` 保持可用。
- MuJoCo 插件适配 Jazzy 的资源管理器初始化流程，无界面运行也会处理 ROS 回调。
- MoveIt 启动配置采用 Jazzy 的规划插件及请求、响应适配器配置，并补充 1 rad/s² 的保守规划加速度限制。
- MoveIt 进程保留 Simple Controller Manager 插件至退出，规避实测的 Jazzy 2.12.4 插件卸载崩溃。
- 关节广播器使用 `joint_state_broadcaster/joint_states`；汇总节点将机械臂和夹爪状态发布到 `/joint_states`，避免订阅自己的输出。

手动加载额外控制器时也要传参数文件，例如在双臂仿真运行后：

```bash
ros2 run controller_manager spawner sim_multi_mode_controller \
  --param-file "$(ros2 pkg prefix franka_bringup)/share/franka_bringup/config/sim/dual_sim_controllers.yaml"
```

## 真机使用

真机需要与 libfranka 0.9.2 匹配的 Panda 系统版本、已启用的 FCI、实时调度权限和满足控制周期的网络。
Docker 使用宿主机内核，因此真机实时内核需要在 Ubuntu 宿主机配置。

### 无夹爪 Panda：只读状态与 RViz

首次连接可以使用独立的只读入口。它以约 30 Hz 读取 FCI 状态，发布七个关节的
`/joint_states`，由 `robot_state_publisher` 生成 TF，并显示无夹爪模型。
它不加载 controller_manager、运动控制器或夹爪节点，不写入阻抗、碰撞阈值或负载参数。

停止其他真机 FCI 客户端，在已构建的容器工作区执行：

```bash
source install/setup.bash
export ROS_DOMAIN_ID=172
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch franka_bringup franka_read_only.launch.py robot_ip:=172.16.1.2
```

IP 按设备实际配置替换。`172` 是本次只读验证使用的独立 ROS 域，用于隔离仿真；
其他终端查看此会话时也需要设置相同环境变量。按 `Ctrl+C` 或关闭 RViz 窗口会结束整个只读会话。
无桌面时加 `use_rviz:=false`。此入口目前固定为无夹爪 Panda，不能用来执行运动。

```bash
ros2 topic echo /joint_states --once
```

Panda 系统 4.2.2 / FCI 服务端 5 / libfranka 0.9.2 的真机只读通信、关节状态和 TF 已实测，
结果见 [验证记录](docs/jazzy-validation.zh-CN.md)。这不代表 ros2_control 真机控制循环已验证。

### 控制入口

单臂真机 MoveIt（无夹爪）：

```bash
source install/setup.bash
export ROS_DOMAIN_ID=172
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch franka_moveit_config moveit.launch.py \
  robot_ip:=172.16.1.2 load_gripper:=false
```

先关闭上述只读会话，避免同一机器人被多个 FCI 客户端占用。
此 MoveIt 入口默认保留机器人现有参数，并将轨迹控制器加载为 inactive：可以看真实姿态和规划，尚不能执行。
现场确认空间清空、人员在机器人旁且停止装置可立即操作后，在另一个相同 ROS 域的容器终端激活：

```bash
ros2 control set_controller_state panda_arm_controller active
```

激活即进入姿态保持控制。RViz 默认规划组为 `panda_arm`，配置的速度/加速度缩放均为 0.05。
执行前检查界面实际值；如果插件恢复成 0.10，请将 Velocity Scaling 和 Accel. Scaling 手动改为 0.05。
首次从 `<current>` 状态出发，只小幅调整末端目标（约 1 cm），先点 Plan 检查整条轨迹，再点 Execute。
规划场景不自动感知现实障碍物；界面 Stop 不是物理停止装置。
结束时先停用控制器，再结束启动进程：

```bash
ros2 control set_controller_state panda_arm_controller inactive
```

`arm_controller_active:=true` 可在启动时直接激活控制器，现场准备完成后才能使用。
`set_default_robot_parameters:=true` 会恢复旧入口写入默认阻抗/碰撞阈值的行为，默认 false。

基础 bringup 入口仍保留：

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=你的机器人IP
```

这个控制入口在硬件初始化时会设置默认阻抗和碰撞阈值；与上面的只读入口不同。
迁移后的真机行为必须在实际硬件上验证。自动验证脚本不会连接机器人，仿真通过也不表示真机实时性能通过。

### libfranka 笛卡尔阻抗体验

本分支的 `cartesian_impedance_demo` 改编自 [libfranka 0.9.2 官方示例](https://github.com/frankarobotics/libfranka/blob/0.9.2/examples/cartesian_impedance_control.cpp)。
它直接使用 libfranka，不通过 MoveIt 或 ros2_control。先停止其他真机 FCI 客户端。

```bash
source install/setup.bash
# 仅查看状态，不启用控制
ros2 run franka_control2 cartesian_impedance_demo 172.16.1.2 --check
# 等待现场用户回车，随后进入阻抗控制
ros2 run franka_control2 cartesian_impedance_demo 172.16.1.2
```

本地改编保留原示例的平移刚度 150 N/m、旋转刚度 10 Nm/rad 和弹簧阻尼控制律，
但不调用 `setDefaultBehavior`，也不提高碰撞阈值或修改负载参数。
以第一个控制周期的实际末端位姿为平衡点，不先移动到预设姿态。
按回车前须确认空间无障碍、停止装置可立即操作；不要握住关节或进入夹挤区域。
开始后轻推末端可感受回复力，`Ctrl+C` 请求结束软件控制，最多 60 秒自动结束。
终端每秒更新进度条、已用时间和剩余时间；显示由独立线程完成，实时回调不打印日志。
软件停止不能替代现场物理停止装置。

该示例与上游一样没有零空间姿态控制，不能保证肘部保持不动，也不是自由拖动示教模式。
该改编已完成编译、只读/取消路径检查；用户在无夹爪 Panda 上运行并提供了到达
100%（60 秒）后正常结束的终端日志。记录见 [真机验证说明](docs/jazzy-validation.zh-CN.md)。
这次体验不覆盖长期稳定性、全部姿态或碰撞保护性能。

## 常见问题

- **无法连接 Docker**：检查 Docker 服务和当前用户对 Docker socket 的访问权限。
- **图形窗口无法打开**：检查宿主机 `DISPLAY`、X11/XWayland 和容器图形权限。可先用 `./run --headless` 验证控制部分。
- **之前创建过无界面容器，后来想打开窗口**：容器的显示挂载在创建时确定。保留工作区卷，停止并删除开发容器后重新运行 `./run`。
- **控制器提示参数缺失**：确认已经重新构建并加载本分支，手动 spawner 命令使用了正确的 `--param-file`。
- **已有 Humble 构建缓存**：使用独立 Jazzy 工作区；不要混用两套发行版的 `build`、`install` 或环境。
- **内存不足**：设置 `CMAKE_BUILD_PARALLEL_LEVEL=2 ./src/tools/build_jazzy`。
