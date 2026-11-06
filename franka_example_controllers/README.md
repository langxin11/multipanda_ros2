# 自定义 Franka 控制器（ROS 2 Jazzy）

先按[中文安装指南](../README.zh-CN.md)完成 Ubuntu 24.04 / Jazzy 环境和工作区构建。
以下命令在容器工作区 `~/multipanda_ws` 中执行；每个新终端先运行
`source install/setup.bash`。原生工作区使用自己的 `install/setup.bash`。

Docker 镜像已安装 `rqt_controller_manager`；原生 Jazzy 环境如缺少该工具，安装
`ros-jazzy-rqt-controller-manager`。查看当前控制器也可以使用：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## 包名、插件名与源码位置

本包的 ROS 包名和 C++ 命名空间都是 `franka_example_controllers`。
插件描述文件是 [franka_example_controllers.xml](franka_example_controllers.xml)。
其中两个现有插件保留兼容别名：

| 配置中的插件类型 | 实际 C++ 类 |
|---|---|
| `franka_controllers/JointImpedanceController` | `franka_example_controllers::JointImpedanceController` |
| `franka_controllers/CartesianImpedanceController` | `franka_example_controllers::CartesianImpedanceController` |

`franka_controllers` 在这里是插件别名的前缀，不是一个可以用 colcon 构建的包。
不要直接替换现有 YAML 中的别名。新增插件建议使用 `franka_example_controllers/你的类名`。

- [src/comless](src/comless)：内置运动或无需目标订阅的示例。
- [src/subscriber](src/subscriber)：接收 ROS 目标消息的控制器。
- [头文件模板](include/franka_example_controllers/subscriber/custom_controller_template.hpp)和
  [源码模板](src/subscriber/custom_controller_template.cpp)：包含待实现的 TODO。
  模板未加入当前 `add_library`，XML 中的 `CustomController` 条目不代表已有可运行实现。

## 创建控制器

1. 在 `include/franka_example_controllers/subscriber/` 和 `src/subscriber/`
   创建头文件及源文件。实现命令接口、状态接口、生命周期回调和 `update()`。
   激活时从当前状态初始化目标；实时更新循环中避免文件写入和阻塞调用。
2. 在 [CMakeLists.txt](CMakeLists.txt) 的 `add_library(${PROJECT_NAME} SHARED ...)`
   中加入 `src/subscriber/my_controller.cpp`。新增依赖同时加入 CMake 和 `package.xml`。
3. 在源文件末尾导出插件，并在 XML 中登记相同的类名。

```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyController,
                       controller_interface::ControllerInterface)
```

在现有 `<library path="franka_example_controllers">` 中添加：

```xml
<class name="franka_example_controllers/MyController"
       type="franka_example_controllers::MyController"
       base_class_type="controller_interface::ControllerInterface">
  <description>Describe the controller's input and behavior.</description>
</class>
```

4. 选择实际启动入口使用的配置文件，登记插件类型及参数。

| 场景 | `franka_bringup` 中的配置路径 |
|---|---|
| 单臂 MuJoCo | `config/sim/single_sim_controllers.yaml` |
| 双臂 MuJoCo | `config/sim/dual_sim_controllers.yaml` |
| 单臂基础真机入口 | `config/real/single_controllers.yaml` |
| 双臂基础真机入口 | `config/real/dual_controllers.yaml` |

MoveIt 使用 `franka_moveit_config/config/` 下的独立配置。不要在未核对 launch
文件时修改另一个场景的 YAML。以下片段需要合并进现有 YAML，不能覆盖整个文件：

```yaml
controller_manager:
  ros__parameters:
    my_controller:
      type: franka_example_controllers/MyController

my_controller:
  ros__parameters:
    arm_id: panda
```

5. 构建并重新加载工作区。修改启动配置后也需要安装 `franka_bringup`。

```bash
./src/tools/build_jazzy --packages-select franka_example_controllers franka_bringup
source install/setup.bash
ros2 launch franka_bringup franka_sim.launch.py unpause:=true
```

在第二个已加载工作区的终端中，先把新控制器加载为 inactive：

```bash
ros2 run controller_manager spawner my_controller --inactive \
  --param-file "$(ros2 pkg prefix franka_bringup)/share/franka_bringup/config/sim/single_sim_controllers.yaml"
ros2 control list_controllers
```

Jazzy 的 spawner 需要显式传入 `--param-file`。确认实现和资源占用后，先停用
占用相同命令接口的旧控制器，再激活新控制器。默认单臂仿真启动的是
`cartesian_impedance_controller`；也可以通过
`ros2 run rqt_controller_manager rqt_controller_manager` 查看和切换状态。
真机入口及当前验证范围见[中文指南](../README.zh-CN.md#真机使用)。

## 现有目标消息接口

### 关节阻抗

`joint_impedance_controller` 订阅 `/joint_impedance/joints_desired`，消息类型为
`std_msgs/msg/Float64MultiArray`。按关节 1 到 7 的顺序提供七个角度，单位 rad。
构造消息的形式如下；`q_target` 应由当前状态和经过检查的轨迹生成：

```python
from std_msgs.msg import Float64MultiArray

msg = Float64MultiArray()
msg.data = list(q_target)  # 七个关节角；该片段只构造消息
```

### 笛卡尔阻抗

`cartesian_impedance_controller` 订阅 `/cartesian_impedance/pose_desired`，消息类型同上。
前 3 项为基坐标系中的位置（米），后 9 项为旋转矩阵的**逐行排列**：

```python
msg = Float64MultiArray()
msg.data = [px, py, pz,
            r11, r12, r13,
            r21, r22, r23,
            r31, r32, r33]
```

这是本控制器的消息协议，不说明四元数本身不稳定；如果目标用四元数表示，需要
先归一化并转换为有效旋转矩阵。代码内部也使用四元数计算姿态误差。
这两个订阅控制器不负责全局避障或完整轨迹规划，发送目标前需要自行处理连续性和范围。

多模式控制器使用另一套消息和服务，参见
[多模式控制器文档](../docs/chapters/franka_multi_mode_controller.md)，不要混用两套接口。
