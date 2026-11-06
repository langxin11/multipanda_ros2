# 学习任务 1：Panda 空中画圆与轨迹分析

目标：独立完成“读取当前状态 → 生成笛卡尔路径 → 规划 → 执行 → 记录 → 分析”。
本例使用单臂 MuJoCo 和 ROS 2 Jazzy，自动生成可展示的 RViz 轨迹和实验图表。
实测结果与展示图见[验证记录](../docs/panda-circle-validation.zh-CN.md)。
当前仅支持仿真；它会核对硬件类型、机器人描述和仿真时间，不提供真机执行开关。

## 运行

按[中文安装指南](../README.zh-CN.md)进入已完成全工作区构建的容器。
以下命令在容器 `~/multipanda_ws` 中执行：

```bash
./src/tools/build_jazzy --packages-select franka_learning_demos
source install/setup.bash
ros2 launch franka_learning_demos circle.launch.py
```

启动后自动运行一圈，默认半径 3 cm，末端朝向保持初始值。窗口显示完成后仍保留，
按 Ctrl+C 退出整个会话。相同 ROS 域中不要再启动另一套 bringup 或 MoveIt。

默认使用独立的 `ROS_DOMAIN_ID=181` 和 `LOCALHOST` 发现范围，避免与既有会话混用。
从另一个容器终端查看本次会话时，先执行：

```bash
source install/setup.bash
export ROS_DOMAIN_ID=181
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic list
```

无界面执行并在完成后关闭所有演示节点：

```bash
ros2 launch franka_learning_demos circle.launch.py gui:=false keep_alive:=false
```

只计算和检查路径，不发送执行请求：

```bash
ros2 launch franka_learning_demos circle.launch.py plan_only:=true
```

每次运行的结果保存在 `log/panda-circle/日期-时间/`，终端会打印绝对路径。
可通过 `output_dir:=绝对路径` 指定一个尚不存在的目录。已有目录不会被覆盖。
若希望从宿主机查看文件，可把输出放在挂载的源码目录中，例如
`output_dir:=/home/developer/multipanda_ws/src/log/circle-study-01`。

## 展示什么

RViz 默认配置展示三个轨迹：白色为请求的圆，青色为控制器参考，橙色为实际反馈。
轨迹接近时可以在左侧 Displays 中关闭其中一条，观察另一条。

本例以 `panda_link0` 为基坐标系，以 `panda_link8` 为观察点。
它是机械臂法兰参考，不是夹爪尖端；当前 MuJoCo 场景仍包含夹爪。
整个圆位于基坐标系的 XY 平面，从当前法兰位置出发，向负 X 方向展开。
圆心为 `(x_start - radius, y_start, z_start)`，因此不会先跳到一个预设起点。

数据链路：

```text
当前关节反馈 → MoveIt FK → 圆形路点 → /compute_cartesian_path
                                               ↓
                                完整路径、时间与状态有效性检查
                                               ↓
                                    /execute_trajectory
                                               ↓
                          /panda_arm_controller/controller_state
                              同一消息中的 reference / feedback
                                               ↓
                                URDF FK → RViz、CSV、误差图
```

MoveIt 2.12.4 的笛卡尔路径服务执行时间参数化。本例还检查返回时间严格递增、
关节数据有效、相邻关节变化不超过 0.2 rad，并检查返回轨迹点的状态有效性。
只有 `fraction` 为完整路径才执行；碰撞检查针对 MoveIt 中建模的环境，
这里不是相机感知、动态避障或连续碰撞检测演示。

每次运行前，本地 URDF 正运动学都会在三个关节配置上与 `/compute_fk` 对照。
这三个查询不驱动机器人。执行时使用控制器同一条状态消息中的目标与反馈计算
位置误差，避免把计划开始的墙钟时间误当成控制器实际开始时间。程序等待控制器、
`/joint_states` 和 MoveIt 状态同步，并在新轨迹时间归零后开始采样，排除先前的姿态保持。
约每 0.01 仿真秒保留一条同步样本；采样覆盖不足 90% 或间断超过 0.25 秒会标记失败。

## 结果文件与判读

| 文件 | 内容 |
|---|---|
| `target.csv` | 请求的几何圆，单位米 |
| `planned.csv` | 规划时间、末端位置、关节位置/速度/加速度；关节顺序见摘要 |
| `tracking.csv` | 同时刻的参考与实际末端位置、误差及七关节原始位置，时间为仿真秒 |
| `summary.json` | 参数、规划比例、执行结果、采样覆盖及误差指标 |
| `trajectory.png` / `trajectory.svg` | 可用于汇报的三维轨迹、XY 投影和误差曲线 |

`summary.json` 中 `completed` 表示执行成功且获得足够的轨迹数据；
`planned_only` 表示只规划，`failed` 或 `cancelled` 不能作为完成证据。
失败时仍尽量保存已有数据和原因。`ros2 launch` 正常退出并不等于实验成功，
应检查这个结果文件。

位置 RMSE 比较 **FK(reference)** 与 **FK(feedback)**，包含执行中的相位滞后；
`circle_distance_rmse_m` 则表示实际点到请求圆的几何距离，不包含沿圆周的相位差。
二者回答的问题不同。这些数值来自仿真反馈和机器人模型，不是外部测得的绝对定位精度。

重新生成图表：

```bash
ros2 run franka_learning_demos plot_circle log/panda-circle/你的结果目录
```

## 你接下来亲自做的三个实验

1. **跑通基准。** 保持默认参数，找到 `fraction`、执行状态、样本数及位置 RMSE。
   截图 RViz，并解释白色、青色、橙色各代表什么。
2. **改变几何。** 把半径从 0.03 改为 0.02，再运行一次。执行前预测：圆心怎么变、
   法兰的 Z 坐标是否变化、夹爪朝向是否变化。用 CSV 验证预测。
3. **改变速度。** 保持半径相同，把 `velocity_scale` 从 0.1 改为 0.05，
   保持 `acceleration_scale` 不变。比较时长、RMSE 和最大误差；不要预设减速后误差必然减半。

```bash
ros2 launch franka_learning_demos circle.launch.py radius:=0.02
ros2 launch franka_learning_demos circle.launch.py velocity_scale:=0.05
```

每次结束上一场仿真后再启动下一场。保存一张实验对比表，列出参数、时长、
RMSE、最大误差和你对差异的解释，即可作为第一次学习成果汇报。

读代码的顺序：

- [circle_demo.py](franka_learning_demos/circle_demo.py) 的 `plan()`：坐标、路点、规划完整性。
- 同文件的 `execute()` / `on_state()`：action 执行与同步记录。
- [kinematics.py](franka_learning_demos/kinematics.py)：从 URDF 构造坐标变换链。
- [report.py](franka_learning_demos/report.py)：几何误差和跟踪误差的区别。

接下来的扩展是“8”字路径、柔顺参数对比、示教点复现，先完成这三个基准实验。
真机迁移需要单独核对工具坐标、模型、负载、轨迹与现场条件，当前 demo 的仿真验证不替代真机验证。

## 开发检查

```bash
colcon test --packages-select franka_learning_demos
colcon test-result --verbose
python3 src/tools/verify_circle_demo.py
```

集成检查覆盖完整画圆、只规划、碰撞阻断拒绝执行、无效半径和非仿真时间拒绝。
虚拟显示器下验证窗口并保存截图：

```bash
xvfb-run -a -s '-screen 0 1600x1000x24' env LIBGL_ALWAYS_SOFTWARE=1 \\
  python3 src/tools/verify_circle_demo.py --gui --screenshot --circle-only
```

参考：[MoveIt 接口教程](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)、
[Jazzy JointTrajectoryController](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)。
接口字段以本项目 Jazzy 容器实际安装版本为准。
