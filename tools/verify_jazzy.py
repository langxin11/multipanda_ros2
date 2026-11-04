#!/usr/bin/env python3
"""Jazzy 集成验证：仅启动 mock 硬件或 MuJoCo，不连接真实机器人。"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=['fake', 'fake_dual', 'single', 'dual', 'moveit', 'moveit_dual', 'all'], default='all')
    parser.add_argument('--gui', action='store_true', help='在当前 DISPLAY 中打开 MuJoCo/RViz 窗口')
    parser.add_argument('--output', default='log/jazzy-verification')
    parser.add_argument('--domain-id', type=int, default=171)
    parser.add_argument('--strict-shutdown', action='store_true',
                        help='子节点退出异常也视为测试失败')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS2CLI_NO_DAEMON'] = '1'
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from ament_index_python.packages import get_package_share_directory

    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    scenarios = ['fake', 'fake_dual', 'single', 'dual', 'moveit', 'moveit_dual'] if args.scenario == 'all' else [args.scenario]
    results = []
    for scenario in scenarios:
        rclpy.init()
        node = rclpy.create_node('jazzy_integration_check')
        process = None
        log_file = (output / f'{scenario}.log').open('w')
        received = {}
        samples = [0]
        expected = [f'{arm}_joint{i}' for arm in (
            ['mj_left', 'mj_right'] if scenario in ('dual', 'moveit_dual') else (['left', 'right'] if scenario == 'fake_dual' else ['panda'])) for i in range(1, 8)]

        def receive(msg):
            for name, position in zip(msg.name, msg.position):
                received[name] = position
            samples[0] += 1

        node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)

        def wait_until(predicate, timeout=60):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if process is not None and process.poll() is not None:
                    raise RuntimeError(f'启动进程提前退出：{process.returncode}')
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            raise TimeoutError('等待控制器或关节反馈超时')

        def service(service_type, name, request, timeout=15):
            client = node.create_client(service_type, name)
            try:
                if not client.wait_for_service(timeout_sec=timeout):
                    raise TimeoutError(f'服务不可用：{name}')
                future = client.call_async(request)
                wait_until(future.done, timeout)
                return future.result()
            finally:
                node.destroy_client(client)

        def controllers():
            reply = service(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
            return {c.name: c.state for c in reply.controller}

        def spawn(name, config, inactive=False):
            command = ['ros2', 'run', 'controller_manager', 'spawner', name,
                       '--param-file', config, '--controller-manager-timeout', '30']
            if inactive:
                command.append('--inactive')
            subprocess.run(command, check=True, timeout=60, stdout=log_file, stderr=subprocess.STDOUT)

        def action(action_type, name, goal, timeout=30):
            from rclpy.action import ActionClient
            client = ActionClient(node, action_type, name)
            try:
                assert client.wait_for_server(timeout_sec=timeout), f'动作服务不可用：{name}'
                future = client.send_goal_async(goal)
                wait_until(future.done, timeout)
                handle = future.result()
                assert handle.accepted, f'动作目标被拒绝：{name}'
                future = handle.get_result_async()
                wait_until(future.done, timeout)
                reply = future.result()
                assert reply.status == 4, f'动作执行失败：{name}，状态 {reply.status}'
                return reply.result
            finally:
                client.destroy()

        try:
            existing = node.create_client(ListControllers, '/controller_manager/list_controllers')
            if existing.wait_for_service(timeout_sec=1):
                raise RuntimeError('测试 ROS_DOMAIN_ID 已有 controller_manager，请更换 --domain-id')
            node.destroy_client(existing)
            if scenario == 'fake':
                command = ['ros2', 'launch', 'franka_bringup', 'franka.launch.py',
                           'use_fake_hardware:=true', 'robot_ip:=unused', f'use_rviz:={str(args.gui).lower()}']
            elif scenario == 'fake_dual':
                command = ['ros2', 'launch', 'franka_bringup', 'dual_franka.launch.py',
                           'use_fake_hardware:=true', 'robot_ip_1:=unused', 'robot_ip_2:=unused',
                           'arm_id_1:=left', 'arm_id_2:=right', f'use_rviz:={str(args.gui).lower()}']
            elif scenario in ('moveit', 'moveit_dual'):
                command = ['ros2', 'launch', 'franka_moveit_config',
                           'sim_dual_moveit.launch.py' if scenario == 'moveit_dual' else 'sim_moveit.launch.py',
                           'unpause:=true', f'headless:={str(not args.gui).lower()}', f'no_render:={str(not args.gui).lower()}', f'use_rviz:={str(args.gui).lower()}']
            else:
                launch = 'dual_franka_sim.launch.py' if scenario == 'dual' else 'franka_sim.launch.py'
                command = ['ros2', 'launch', 'franka_bringup', launch,
                           'unpause:=true', f'headless:={str(not args.gui).lower()}', f'no_render:={str(not args.gui).lower()}', f'use_rviz:={str(args.gui).lower()}']
            process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True)
            wait_until(lambda: all(name in received for name in expected) and samples[0] >= 10)
            assert all(math.isfinite(received[name]) for name in expected), '关节位置含非有限值'
            wait_until(lambda: controllers().get('joint_state_broadcaster') == 'active', 30)
            states = controllers()
            assert states.get('joint_state_broadcaster') == 'active', states
            evidence = {'joints': len(expected), 'initial_controllers': states}
            if scenario == 'single':
                wait_until(lambda: controllers().get('cartesian_impedance_controller') == 'active', 30)
                config = str(Path(get_package_share_directory('franka_bringup')) / 'config/sim/single_sim_controllers.yaml')
                spawn('joint_impedance_controller', config, inactive=True)
                request = SwitchController.Request()
                request.activate_controllers = ['joint_impedance_controller']
                request.deactivate_controllers = ['cartesian_impedance_controller']
                request.strictness = SwitchController.Request.STRICT
                request.timeout.sec = 10
                assert service(SwitchController, '/controller_manager/switch_controller', request).ok
                # Confirm that commands reach the simulated plant, not just that nodes exist.
                before = received['panda_joint1']
                goal = Float64MultiArray()
                goal.data = [received[name] for name in expected]
                goal.data[0] += 0.05
                publisher = node.create_publisher(Float64MultiArray, '/joint_impedance/joints_desired', 10)
                wait_until(lambda: publisher.get_subscription_count() > 0, 10)
                publisher.publish(goal)
                wait_until(lambda: received['panda_joint1'] - before > 0.01, 15)
                evidence['joint1_motion_rad'] = received['panda_joint1'] - before
                node.destroy_publisher(publisher)
            elif scenario == 'dual':
                config = str(Path(get_package_share_directory('franka_bringup')) / 'config/sim/dual_sim_controllers.yaml')
                spawn('sim_multi_mode_controller', config)
                assert controllers().get('sim_multi_mode_controller') == 'active'
                from multi_mode_control_msgs.srv import GetControllers
                response = service(GetControllers, '/sim_multi_mode_controller/get_controllers', GetControllers.Request())
                evidence['controllets'] = [{'name': c.name, 'resources': list(c.resources)} for c in response.controllers]
                assert response.controllers, '双臂多模式控制器未激活 controllet'
                from multi_mode_control_msgs.srv import SetControllers
                from multi_mode_control_msgs.msg import Controller, JointGoal
                request = SetControllers.Request()
                request.controllers = [Controller(name='panda_joint_impedance_controller', resources=[arm])
                                       for arm in ['mj_left', 'mj_right']]
                service(SetControllers, '/sim_multi_mode_controller/set_controllers', request)
                def joint_controllets_active():
                    reply = service(GetControllers, '/sim_multi_mode_controller/get_controllers', GetControllers.Request())
                    return len(reply.controllers) == 2 and all(
                        c.name == 'panda_joint_impedance_controller' for c in reply.controllers)
                wait_until(joint_controllets_active, 10)
                movements = {}
                for arm, direction in [('mj_left', 1.0), ('mj_right', -1.0)]:
                    before = received[f'{arm}_joint1']
                    goal = JointGoal()
                    goal.q = [received[f'{arm}_joint{i}'] for i in range(1, 8)]
                    goal.q[0] += direction * 0.03
                    publisher = node.create_publisher(
                        JointGoal, f'/{arm}/panda_joint_impedance_controller/desired_pose', 10)
                    wait_until(lambda: publisher.get_subscription_count() > 0, 10)
                    publisher.publish(goal)
                    wait_until(lambda: direction * (received[f'{arm}_joint1'] - before) > 0.005, 15)
                    movements[arm] = received[f'{arm}_joint1'] - before
                    node.destroy_publisher(publisher)
                evidence['joint_controllets_motion_rad'] = movements
            elif scenario in ('moveit', 'moveit_dual'):
                from moveit_msgs.srv import GetMotionPlan
                from moveit_msgs.msg import Constraints, JointConstraint
                from moveit_msgs.action import ExecuteTrajectory
                controller = 'dual_panda_arm_controller' if scenario == 'moveit_dual' else 'panda_arm_controller'
                wait_until(lambda: controllers().get(controller) == 'active', 30)
                request = GetMotionPlan.Request()
                req = request.motion_plan_request
                req.group_name = 'dual_panda' if scenario == 'moveit_dual' else 'panda_arm'
                req.pipeline_id = 'ompl'
                req.allowed_planning_time = 10.0
                req.max_velocity_scaling_factor = 0.1
                req.max_acceleration_scaling_factor = 0.1
                req.start_state.is_diff = True
                constraints = Constraints()
                target_joint = expected[0]
                target = received[target_joint] + 0.05
                for name in expected:
                    constraints.joint_constraints.append(JointConstraint(
                        joint_name=name, position=target if name == target_joint else received[name],
                        tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
                req.goal_constraints = [constraints]
                plan = service(GetMotionPlan, '/plan_kinematic_path', request, timeout=30).motion_plan_response
                assert plan.error_code.val == 1, f'MoveIt 规划失败：{plan.error_code.val}'
                assert plan.trajectory.joint_trajectory.points
                goal = ExecuteTrajectory.Goal()
                goal.trajectory = plan.trajectory
                result = action(ExecuteTrajectory, '/execute_trajectory', goal)
                assert result.error_code.val == 1, f'MoveIt 执行失败：{result.error_code.val}'
                wait_until(lambda: abs(received[target_joint] - target) < 0.02, 10)
                evidence['target_joint'] = target_joint
                evidence['trajectory_points'] = len(plan.trajectory.joint_trajectory.points)
                evidence['joint1_target_error_rad'] = abs(received[target_joint] - target)
            if scenario == 'single':
                from franka_msgs.action import Move
                goal = Move.Goal(width=0.04, speed=0.1)
                result = action(Move, '/panda_gripper_sim_node/move', goal)
                assert result.success, result.error
                evidence['gripper_move'] = True
            count = samples[0]
            wait_until(lambda: samples[0] >= count + 20, 10)
            assert all(math.isfinite(received[name]) for name in expected)
            results.append({'scenario': scenario, 'passed': True, **evidence})
            print(f'功能通过：{scenario}，{len(expected)} 个关节反馈有效', flush=True)
        except Exception as error:
            results.append({'scenario': scenario, 'passed': False, 'error': str(error)})
            print(f'失败：{scenario}：{error}；日志：{output / (scenario + ".log")}', flush=True)
        finally:
            if process is not None and process.poll() is None:
                # Let ros2 launch deliver SIGINT to its children once.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            log_file.close()
            node.destroy_node()
            rclpy.shutdown()
        launch_log = (output / f'{scenario}.log').read_text(errors='replace')
        runtime_log = launch_log.split('user interrupted', 1)[0]
        if 'process has died' in runtime_log:
            results[-1]['passed'] = False
            results[-1]['error'] = '运行期间有子节点异常退出，详见启动日志'
        shutdown_errors = [
            {'node': name, 'exit_code': int(code)}
            for name, code in re.findall(
                r'\[ERROR\] \[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)', launch_log)
        ]
        results[-1]['shutdown_clean'] = not shutdown_errors
        if shutdown_errors:
            results[-1]['shutdown_errors'] = shutdown_errors
            print(f'退出检查异常：{scenario}，子节点异常 {shutdown_errors}', flush=True)
        (output / 'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')
    return 0 if all(r['passed'] and (not args.strict_shutdown or r['shutdown_clean']) for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
