"""Plan one circle in MuJoCo and record synchronized controller reference/feedback."""
import copy
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListHardwareComponents
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPlanningScene, GetPositionFK, GetStateValidity
from nav_msgs.msg import Path as RosPath
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from .kinematics import SerialChain, quaternion_matrix
from .report import write_report


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CircleDemo(Node):
    def __init__(self):
        super().__init__('panda_circle_demo')
        defaults = {
            'radius': 0.03, 'waypoints': 80, 'velocity_scale': 0.1,
            'acceleration_scale': 0.1, 'eef_step': 0.003,
            'plan_only': False, 'keep_alive': True,
            'output_dir': 'log/panda-circle/' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'),
            'execution_timeout': 120.0,
        }
        self.config = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.base, self.tip = 'panda_link0', 'panda_link8'
        self.names = [f'panda_joint{i}' for i in range(1, 8)]
        self.latest = None
        self.joint_state = None
        self.last_received = 0.0
        self.chain = None
        self.recording = False
        self.trajectory_started = False
        self.record_error = None
        self.goal_handle = None
        self.samples = []
        self.target = []
        self.plan_rows = []
        self.duration = 0.0
        self.summary = {
            'status': 'starting', 'simulation_only': True,
            'created_at': datetime.now().astimezone().isoformat(),
            'frame': self.base, 'tip': self.tip,
            'config': {key: repr(value) if isinstance(value, float) and not math.isfinite(value) else value
                       for key, value in self.config.items()},
            'execution_success': False, 'fraction': None,
            'measurement': 'FK of synchronized JTC reference and feedback; not external metrology',
        }
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_publishers = {name: self.create_publisher(RosPath, f'~/paths/{name}', qos)
                           for name in ('target', 'reference', 'actual')}
        self.paths = {name: RosPath() for name in self.path_publishers}
        for path in self.paths.values():
            path.header.frame_id = self.base
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState, '/panda_arm_controller/controller_state',
            self.on_state, qos_profile_sensor_data)
        self.joint_subscription = self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.publish_paths)
        self.action = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def wait(self, future, timeout=30.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError('等待 ROS 响应超时')
        result = future.result()
        if result is None:
            raise RuntimeError('ROS 请求没有返回结果')
        return result

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.create_client(srv_type, name)
        try:
            deadline = time.monotonic() + timeout
            while not client.wait_for_service(timeout_sec=0.2):
                rclpy.spin_once(self, timeout_sec=0.02)
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'服务不可用：{name}')
            return self.wait(client.call_async(request), timeout)
        finally:
            self.destroy_client(client)

    def validate_config(self):
        for key, low, high in [('radius', 0.005, 0.08), ('velocity_scale', 0.01, 0.3),
                               ('acceleration_scale', 0.01, 0.3), ('eef_step', 0.0005, 0.005),
                               ('execution_timeout', 10., 600.)]:
            value = self.config[key]
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{key} 必须在 [{low}, {high}] 范围内')
        if not 20 <= self.config['waypoints'] <= 720:
            raise ValueError('waypoints 必须在 [20, 720] 范围内')
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('本例仅用于 MuJoCo，必须启用 use_sim_time')

    def assert_simulation(self):
        hardware = self.call(ListHardwareComponents, '/controller_manager/list_hardware_components',
                             ListHardwareComponents.Request())
        # Imported MuJoCo components on Jazzy leave plugin_name empty. Require the
        # component name/state AND verify the live URDF's plugin below.
        if (len(hardware.component) != 1 or
                hardware.component[0].name != 'FrankaMjHardwareSystem' or
                hardware.component[0].plugin_name not in ('', 'franka_hardware/FrankaMjHardwareSystem') or
                hardware.component[0].state.label != 'active'):
            raise RuntimeError('检测到的硬件不是单个 FrankaMjHardwareSystem，拒绝执行')
        params = self.call(GetParameters, '/robot_state_publisher/get_parameters',
                           GetParameters.Request(names=['robot_description', 'use_sim_time']))
        description = params.values[0].string_value
        plugins = [item.text for item in ET.fromstring(description).findall('ros2_control/hardware/plugin')]
        if plugins != ['franka_hardware/FrankaMjHardwareSystem'] or not params.values[1].bool_value:
            raise RuntimeError('机器人描述或时间源不符合 MuJoCo 配置')
        params = self.call(GetParameters, '/move_group/get_parameters',
                           GetParameters.Request(names=['use_sim_time']))
        if not params.values[0].bool_value:
            raise RuntimeError('MoveIt 没有使用仿真时间')
        self.chain = SerialChain(description, self.base, self.tip)
        if self.chain.joint_names != self.names:
            raise RuntimeError('运动链与预期的单臂 Panda 不一致')
        self.summary['hardware_plugin'] = plugins[0]

    def await_state(self):
        deadline = time.monotonic() + 30.0
        stable_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None and self.joint_state is not None and time.monotonic() - self.last_received < 0.5:
                published = dict(zip(self.joint_state.name, self.joint_state.position))
                observed = dict(zip(self.latest.joint_names, self.latest.feedback.positions))
                if (not all(name in published and math.isfinite(published[name]) for name in self.names) or
                        max(abs(published[name] - observed[name]) for name in self.names) > 0.002 or
                        abs(self.get_clock().now().nanoseconds * 1e-9 - seconds(self.joint_state.header.stamp)) > 0.5):
                    stable_since = None
                    continue
                velocity = self.latest.feedback.velocities
                if len(velocity) == 7 and np.all(np.isfinite(velocity)) and max(abs(v) for v in velocity) < 0.01:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since > 0.5:
                        return
                else:
                    stable_since = None
        raise TimeoutError('没有收到稳定、新鲜的控制器反馈；确认仿真已取消暂停')

    def on_joint_state(self, msg):
        self.joint_state = msg

    def state(self, positions, names=None):
        return RobotState(joint_state=JointState(name=names or self.names, position=list(positions)), is_diff=True)

    def pose_from_moveit(self, positions):
        response = self.call(GetPositionFK, '/compute_fk', GetPositionFK.Request(
            header=self.paths['target'].header, fk_link_names=[self.tip],
            robot_state=self.state(positions)))
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise RuntimeError('MoveIt 正运动学查询失败')
        return response.pose_stamped[0].pose

    def check_fk(self, q):
        # Compare independent implementations at the start and two nearby configurations.
        max_error = 0.0
        for delta in (0., 0.02, -0.02):
            sample = list(q)
            sample[0] += delta
            sample[3] += delta
            pose = self.pose_from_moveit(sample)
            transform = self.chain.fk(dict(zip(self.names, sample)))
            error = max(np.linalg.norm(transform[:3, 3] - [pose.position.x, pose.position.y, pose.position.z]),
                        np.linalg.norm(transform[:3, :3] - quaternion_matrix(pose.orientation)))
            max_error = max(max_error, float(error))
        self.summary['fk_crosscheck_max_error'] = max_error
        if max_error > 1e-6:
            raise RuntimeError('URDF 正运动学与 MoveIt 不一致，拒绝记录错误数据')

    def on_state(self, msg):
        if set(msg.joint_names) != set(self.names):
            return
        if len(msg.feedback.positions) != 7 or not np.all(np.isfinite(msg.feedback.positions)):
            self.record_error = '控制器反馈缺失或包含非有限数值'
            return
        self.latest = msg
        self.last_received = time.monotonic()
        if not self.recording:
            return
        t = seconds(msg.reference.time_from_start)
        if t <= 0.0 or t > self.duration:
            return
        if not self.trajectory_started:
            # ExecuteTrajectory is accepted before JTC adopts the new trajectory.
            # Exclude the old hold trajectory (already running while we awaited
            # stable feedback and planned). A new trajectory starts near zero.
            if t > 0.25:
                return
            self.trajectory_started = True
        # The controller may publish at 1 kHz. Keep synchronized samples at
        # approximately 100 Hz without flooding RViz with growing paths.
        if self.samples and t - self.samples[-1][1] < 0.01 - 1e-9:
            return
        try:
            if len(msg.reference.positions) != 7:
                raise ValueError('控制器目标维度不匹配')
            ref = self.chain.fk(dict(zip(msg.joint_names, msg.reference.positions)))
            actual = self.chain.fk(dict(zip(msg.joint_names, msg.feedback.positions)))
            cosine = np.clip((np.trace(ref[:3, :3].T @ actual[:3, :3]) - 1) / 2, -1, 1)
            row = [seconds(msg.header.stamp), t, *ref[:3, 3], *actual[:3, 3],
                   float(np.linalg.norm(actual[:3, 3] - ref[:3, 3])), math.acos(cosine),
                   *[msg.reference.positions[msg.joint_names.index(name)] for name in self.names],
                   *[msg.feedback.positions[msg.joint_names.index(name)] for name in self.names]]
            self.samples.append(row)
            for name, transform in [('reference', ref), ('actual', actual)]:
                point = PoseStamped()
                point.header.frame_id = self.base
                point.header.stamp = msg.header.stamp
                point.pose.position.x, point.pose.position.y, point.pose.position.z = map(float, transform[:3, 3])
                point.pose.orientation.w = 1.0
                self.paths[name].poses.append(point)
        except (ValueError, KeyError) as exc:
            self.record_error = str(exc)

    def publish_paths(self):
        for name, publisher in self.path_publishers.items():
            self.paths[name].header.stamp = self.get_clock().now().to_msg()
            publisher.publish(self.paths[name])

    def plan(self):
        q = [self.latest.feedback.positions[self.latest.joint_names.index(name)] for name in self.names]
        self.check_fk(q)
        origin = self.pose_from_moveit(q)
        radius = self.config['radius']
        self.summary['center_m'] = [origin.position.x - radius, origin.position.y, origin.position.z]
        waypoints = []
        for theta in np.linspace(0., 2.*math.pi, self.config['waypoints'] + 1):
            pose = copy.deepcopy(origin)
            pose.position.x += radius * (math.cos(theta) - 1.)
            pose.position.y += radius * math.sin(theta)
            waypoints.append(pose)
            self.target.append([pose.position.x, pose.position.y, pose.position.z])
            self.paths['target'].poses.append(PoseStamped(header=self.paths['target'].header, pose=pose))
        self.publish_paths()
        request = GetCartesianPath.Request(
            header=self.paths['target'].header, start_state=self.state(q),
            group_name='panda_arm', link_name=self.tip, waypoints=waypoints,
            max_step=self.config['eef_step'], avoid_collisions=True,
            max_velocity_scaling_factor=self.config['velocity_scale'],
            max_acceleration_scaling_factor=self.config['acceleration_scale'])
        result = self.call(GetCartesianPath, '/compute_cartesian_path', request)
        self.summary['fraction'] = result.fraction
        if result.error_code.val != MoveItErrorCodes.SUCCESS or not math.isfinite(result.fraction) or result.fraction < 1. - 1e-6:
            raise RuntimeError(f'笛卡尔路径不完整（fraction={result.fraction:.6f}），没有执行')
        trajectory = result.solution.joint_trajectory
        if set(trajectory.joint_names) != set(self.names) or len(trajectory.points) < 2:
            raise RuntimeError('规划轨迹的关节或轨迹点不完整')
        previous_time = -1.0
        previous_q = np.array([q[self.names.index(name)] for name in trajectory.joint_names])
        for point in trajectory.points:
            t = seconds(point.time_from_start)
            if len(point.positions) != 7 or not np.all(np.isfinite(point.positions)):
                raise RuntimeError('规划轨迹位置无效')
            if t <= previous_time or t < 0.0:
                raise RuntimeError('规划轨迹缺少严格递增的时间参数')
            if np.max(np.abs(np.array(point.positions) - previous_q)) > 0.2:
                raise RuntimeError('规划轨迹存在超过 0.2 rad 的相邻关节跳变')
            for values in (point.velocities, point.accelerations):
                if len(values) != 7 or not np.all(np.isfinite(values)):
                    raise RuntimeError('规划轨迹速度或加速度无效')
            # Time parameterization can resample the IK path: check the returned states too.
            validity = self.call(GetStateValidity, '/check_state_validity', GetStateValidity.Request(
                robot_state=self.state(point.positions, trajectory.joint_names), group_name='panda_arm'))
            if not validity.valid:
                raise RuntimeError('时间参数化后的轨迹点未通过状态有效性检查')
            transform = self.chain.fk(dict(zip(trajectory.joint_names, point.positions)))
            self.plan_rows.append([t, *transform[:3, 3], *point.positions,
                                   *point.velocities, *point.accelerations])
            previous_time, previous_q = t, np.array(point.positions)
        self.duration = seconds(trajectory.points[-1].time_from_start)
        if self.duration <= 0.0:
            raise RuntimeError('规划轨迹时长无效')
        self.summary.update(planned_duration_s=self.duration, trajectory_points=len(trajectory.points))
        self.summary['planned_joint_names'] = list(trajectory.joint_names)
        self.get_logger().info(f'圆轨迹规划完成：{len(trajectory.points)} 点，仿真时长 {self.duration:.2f} 秒')
        return result.solution, q

    def execute(self, trajectory, start_q):
        self.await_state()
        current = [self.latest.feedback.positions[self.latest.joint_names.index(name)] for name in self.names]
        if np.max(np.abs(np.array(current) - start_q)) > 0.01:
            raise RuntimeError('规划后机器人起点已改变，需重新规划')
        self.assert_simulation()
        # JTC can become active before joint_state_broadcaster. Wait until
        # MoveIt's state monitor agrees with the controller, not just its FK service.
        deadline = time.monotonic() + 10.
        while True:
            scene = self.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
                components=PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)))
            joints = scene.scene.robot_state.joint_state
            positions = dict(zip(joints.name, joints.position))
            if (all(name in positions and math.isfinite(positions[name]) for name in self.names) and
                    max(abs(positions[name] - start_q[i]) for i, name in enumerate(self.names)) < 0.005):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('MoveIt 当前状态尚未与控制器反馈同步，未发送执行请求')
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.action.wait_for_server(timeout_sec=10.):
            raise TimeoutError('MoveIt 执行动作不可用')
        self.samples.clear()
        self.trajectory_started = False
        self.recording = True
        self.goal_handle = self.wait(self.action.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory, controllers=['panda_arm_controller'])))
        if not self.goal_handle.accepted:
            raise RuntimeError('MoveIt 拒绝执行请求')
        future = self.goal_handle.get_result_async()
        deadline = time.monotonic() + self.config['execution_timeout']
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.record_error:
                raise RuntimeError(self.record_error)
            if time.monotonic() - self.last_received > 2.:
                raise RuntimeError('执行时控制器反馈中断')
            if time.monotonic() > deadline:
                raise TimeoutError('执行超过墙钟时间限制，已请求取消')
        result = future.result()
        self.goal_handle = None
        # Drain the final controller-state publications after the action result.
        until = time.monotonic() + 0.2
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.02)
        self.recording = False
        self.summary['execute_error_code'] = result.result.error_code.val
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'MoveIt 执行未成功：{result.result.error_code.val}')
        self.summary['execution_success'] = True
        if self.record_error or len(self.samples) < 20:
            raise RuntimeError(self.record_error or '有效轨迹记录少于 20 个样本')
        coverage = (self.samples[-1][1] - self.samples[0][1]) / self.duration
        self.summary['recorded_time_coverage'] = coverage
        if coverage < 0.9:
            raise RuntimeError('采集未覆盖至少 90% 的轨迹时段')
        if max(np.diff(np.array(self.samples)[:, 1])) > 0.25:
            raise RuntimeError('采样间断超过 0.25 仿真秒，记录不完整')
        self.summary['status'] = 'completed'

    def cancel(self):
        if self.goal_handle is not None and self.goal_handle.accepted:
            try:
                self.wait(self.goal_handle.cancel_goal_async(), timeout=3.)
            except Exception as exc:
                self.get_logger().error(f'取消请求未确认：{exc}')
        self.recording = False

    def save(self, directory):
        for filename, header, rows in [
                ('target.csv', ['x_m', 'y_m', 'z_m'], self.target),
                ('planned.csv', ['time_s', 'x_m', 'y_m', 'z_m'] +
                 [f'{kind}_{i}' for kind in ('q_rad', 'dq_rad_s', 'ddq_rad_s2') for i in range(1, 8)],
                 self.plan_rows),
                ('tracking.csv', ['stamp_s', 'trajectory_time_s', 'ref_x_m', 'ref_y_m', 'ref_z_m',
                                  'actual_x_m', 'actual_y_m', 'actual_z_m', 'position_error_m',
                                  'orientation_error_rad'] +
                 [f'{kind}_q{i}_rad' for kind in ('ref', 'actual') for i in range(1, 8)], self.samples)]:
            with (directory / filename).open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(header)
                writer.writerows(rows)
        self.summary['samples'] = len(self.samples)
        (directory / 'summary.json').write_text(json.dumps(self.summary, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
        write_report(directory)
        self.get_logger().info(f'结果已保存：{directory}')


def main(args=None):
    # ros2 launch can inherit an ignored SIGINT from its parent. Install Python's
    # handler explicitly so cancellation works before and after trajectory execution.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CircleDemo()
    directory = Path(node.config['output_dir']).expanduser().resolve()
    exit_code = 0
    try:
        # Each run gets a new directory, so failed runs cannot overwrite earlier evidence.
        directory.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        node.get_logger().error(f'结果目录已存在，拒绝覆盖：{directory}')
        node.destroy_node()
        rclpy.shutdown()
        return 2
    try:
        node.validate_config()
        node.get_logger().info('等待单臂 MuJoCo、MoveIt 和稳定的控制器反馈……')
        node.assert_simulation()
        node.await_state()
        trajectory, q = node.plan()
        if node.config['plan_only']:
            node.summary['status'] = 'planned_only'
        else:
            node.execute(trajectory, q)
    except KeyboardInterrupt:
        node.cancel()
        node.summary.update(status='cancelled', reason='用户中止')
        exit_code = 130
    except Exception as exc:
        node.cancel()
        node.summary.update(status='failed', reason=str(exc))
        node.get_logger().error(str(exc))
        exit_code = 1
    try:
        node.save(directory)
        node.publish_paths()
        if exit_code == 0 and node.config['keep_alive']:
            node.get_logger().info('演示完成，保留轨迹显示；Ctrl+C 结束。')
            # A wall-time timeout keeps Python signal handling responsive even
            # after /clock stops and no simulation-time timers can wake the executor.
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(f'保存或展示结果失败：{exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code
