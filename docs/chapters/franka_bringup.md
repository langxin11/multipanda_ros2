# franka_bringup

This chapter describes the Ubuntu 24.04 / ROS 2 Jazzy branch. Complete the
[installation and build guide](../../README.zh-CN.md) first. Commands below run
from the container workspace after `source install/setup.bash`.

## Launch files and controller configurations

The installed launch files retain their filenames, even though the source files
are organized into `launch/real`, `launch/sim`, and `launch/mixed`.

| Entry point | Configuration under `franka_bringup/config/` |
|---|---|
| `franka.launch.py` | `real/single_controllers.yaml` |
| `dual_franka.launch.py` | `real/dual_controllers.yaml` |
| `multimode_franka.launch.py` | `real/single_multimode.yaml` |
| `dual_multimode_franka.launch.py` | `real/dual_multimode.yaml` |
| `franka_sim.launch.py` | `sim/single_sim_controllers.yaml` |
| `dual_franka_sim.launch.py` | `sim/dual_sim_controllers.yaml` |

MoveIt has separate entry points and controller YAMLs in `franka_moveit_config`.
The read-only entry point `franka_read_only.launch.py` does not load a controller
manager. See the [real robot guide](../../README.zh-CN.md#真机使用) for these workflows.

### Real robot arguments

- Single-arm basic bringup: `robot_ip`, `load_gripper`, `use_rviz`,
  `use_fake_hardware`, `fake_sensor_commands`. The arm name is `panda`.
- Dual-arm bringup: `robot_ip_1`, `robot_ip_2`, `arm_id_1`, `arm_id_2`,
  `load_gripper_1`, `load_gripper_2`, plus the common visualization/mock arguments.
- Single-arm multimode bringup uses the suffixed names `robot_ip_1`, `arm_id_1`,
  and `load_gripper_1` as well.

`load_gripper` is a launch argument; `hand` is the underlying xacro argument.
They are not interchangeable at the command line. On Jazzy,
`use_fake_hardware:=true` selects `mock_components/GenericSystem`. Mock hardware
is useful for interface checks; MuJoCo provides the physics simulation.

Inspect an entry point without launching it:

```bash
ros2 launch franka_bringup franka.launch.py --show-args
ros2 launch franka_bringup franka_sim.launch.py --show-args
```

### MuJoCo arguments

Single-arm simulation uses `arm_id` (default `panda`), `initial_positions`, and
`use_rviz`. Dual-arm simulation uses `arm_id_1` / `arm_id_2` (defaults `mj_left` /
`mj_right`) and `initial_positions_1` / `initial_positions_2`.
The current launch files fix gripper inclusion in Python to match the XML scene;
`hand:=false` is not a supported launch switch for removing the simulated hand.

The included MuJoCo launch also provides `unpause`, `headless`, `no_render`, and
`realtime`. Simulation starts paused unless `unpause:=true` is supplied.
`headless` controls the MuJoCo viewer; `use_rviz` independently controls RViz.

```bash
ros2 launch franka_bringup franka_sim.launch.py unpause:=true use_rviz:=true
```

For a terminal-only session:

```bash
ros2 launch franka_bringup franka_sim.launch.py \
  unpause:=true headless:=true no_render:=true use_rviz:=false
```

The single-arm scene is `franka_description/mujoco/franka/scene.xml`; the dual-arm
scene is `dual_scene.xml`. URDF/xacro names, MuJoCo joint/body/actuator names and
controller `arm_id` values must agree. Renaming a launch argument alone does not
rename the XML model. See [robot descriptions](franka_description.md).

## Node structure

Real-robot launch files start Jazzy's `controller_manager/ros2_control_node`.
The retained `franka_control2_node` executable forwards to that same node for
compatibility. In simulation, `mujoco_ros/mujoco_node` loads the
`mujoco_ros2_control::MujocoRos2ControlPlugin`, which hosts the controller manager.
The include is [launch_mujoco_ros_server.launch](../../franka_bringup/launch/sim/launch_mujoco_ros_server.launch).
It leaves the node name unset so a global name remapping is not applied to the
embedded controller-manager nodes.

`robot_state_publisher` publishes TF from the robot description and joint states.
`joint_state_broadcaster` publishes hardware feedback on its local
`joint_state_broadcaster/joint_states` topic. `joint_state_publisher` aggregates
that feedback and the gripper states into `/joint_states`; it does not command
joint motion.

Real and simulated robots share joint torque/position/velocity interfaces.
Direct Cartesian hardware command interfaces are available only on real robots;
a Cartesian impedance controller that outputs joint torques can also run in MuJoCo.

## Adding a controller on Jazzy

Register the plugin type under `controller_manager.ros__parameters`, and place
its parameters in a separate root block in the selected YAML:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
    my_controller:
      type: my_controller_package/MyController

my_controller:
  ros__parameters:
    arm_id: panda
```

This declares a controller; a spawner must still load/configure/activate it.
On Jazzy, pass the same YAML explicitly to the spawner:

```python
Node(
    package='controller_manager',
    executable='spawner',
    arguments=['my_controller', '--inactive', '--param-file', controllers_yaml],
    output='screen',
)
```

`controllers_yaml` is the installed absolute path for the selected configuration.
Use `--inactive` when loading a controller for inspection before activation.
Controllers claiming the same command interfaces must be switched, rather than
activated together. See the [custom controller tutorial](../../franka_example_controllers/README.md).

Simulation YAMLs additionally contain the `mujoco_server.ros__parameters.MujocoPlugins`
block. Preserve this block when adding controllers. The current example is
[single_sim_controllers.yaml](../../franka_bringup/config/sim/single_sim_controllers.yaml).

## Other chapters

- [Back to main](../main.md)
- [franka_description](franka_description.md)
- [franka_hardware](franka_hardware.md)
- [franka_multi_mode_controller](franka_multi_mode_controller.md)
- [garmi_packages](garmi.md)
