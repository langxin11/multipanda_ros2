#!/usr/bin/env python3
"""Validate the circle learning demo in fresh MuJoCo sessions; never use real hardware."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


def run(command, logfile, env, ready=None, screenshot=None):
    with logfile.open('w') as stream:
        process = subprocess.Popen(command, env=env, stdin=subprocess.DEVNULL,
                                   stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 150
            while process.poll() is None and time.monotonic() < deadline:
                if ready and ready.exists():
                    time.sleep(1.)
                    if screenshot:
                        from PIL import ImageGrab
                        ImageGrab.grab().save(screenshot)
                    return
                time.sleep(0.2)
            if process.poll() is None:
                raise TimeoutError(f'Timed out: {logfile}')
        finally:
            stop(process)


def launch_case(root, name, extra, domain, gui=False, screenshot=False):
    destination = root / name
    args = ['ros2', 'launch', '-n', 'franka_learning_demos', 'circle.launch.py',
            f'gui:={str(gui).lower()}', f'keep_alive:={str(gui).lower()}',
            f'output_dir:={destination}', f'domain_id:={domain}', *extra]
    run(args, root / f'{name}.log', os.environ,
        ready=destination / 'trajectory.svg' if gui else None,
        screenshot=root / 'gui.png' if screenshot else None)
    result = json.loads((destination / 'summary.json').read_text())
    logs = (root / f'{name}.log').read_text()
    result['shutdown_clean'] = 'failed to terminate' not in logs and 'process has died' not in logs
    result['shutdown_escalated'] = 'failed to terminate' in logs
    result['demo_shutdown_clean'] = bool(re.search(r'\[circle_demo-\d+\]: process has finished cleanly', logs))
    return result


def blocked_case(root, domain):
    """Put a collision object over the initial workspace, then require refusal."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    with (root / 'blocked-simulation.log').open('w') as log:
        simulation = subprocess.Popen([
            'ros2', 'launch', '-n', 'franka_moveit_config', 'sim_moveit.launch.py', 'unpause:=true',
            'headless:=true', 'no_render:=true', 'use_rviz:=false'], env=env,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            # A subprocess gets the same isolated domain before rclpy initialization.
            injection = '''
import rclpy
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
rclpy.init()
node = rclpy.create_node('circle_test_obstacle')
client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
assert client.wait_for_service(timeout_sec=40.)
request = ApplyPlanningScene.Request()
request.scene.is_diff = True
request.scene.robot_state.is_diff = True
box = CollisionObject(id='circle_test_obstacle', operation=CollisionObject.ADD)
box.header.frame_id = 'panda_link0'
box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.5, 0.5, 0.5])]
pose = Pose()
pose.orientation.w = 1.
pose.position.x, pose.position.z = 0.3, 0.6
box.primitive_poses = [pose]
request.scene.world.collision_objects = [box]
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
assert future.result().success
node.destroy_node()
rclpy.shutdown()
'''
            subprocess.run(['python3', '-c', injection], env=env, check=True, timeout=60,
                           stdout=log, stderr=subprocess.STDOUT)
            target = root / 'blocked'
            run(['ros2', 'run', 'franka_learning_demos', 'circle_demo', '--ros-args',
                 '-p', 'use_sim_time:=true', '-p', 'keep_alive:=false', '-p', f'output_dir:={target}'],
                root / 'blocked.log', env)
            result = json.loads((target / 'summary.json').read_text())
            assert result['status'] == 'failed' and result['fraction'] is not None and result['fraction'] < 1.
            assert not result['execution_success'] and result['samples'] == 0
            return result
        finally:
            stop(simulation)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('log/circle-verification') / datetime.now().strftime('%Y%m%d-%H%M%S'))
    parser.add_argument('--domain-id', type=int, default=183)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--circle-only', action='store_true', help='Run only the motion/display check')
    parser.add_argument('--screenshot', action='store_true', help='Capture the test display; use with Xvfb and --gui')
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    results = {}
    full = launch_case(root, 'circle', [], args.domain_id, args.gui, args.screenshot)
    assert full['status'] == 'completed' and full['execution_success'] and full['fraction'] == 1.
    assert full['demo_shutdown_clean'] and not full['shutdown_escalated'], 'Demo exit failed or a child required forced termination'
    assert 0.9 <= full['recorded_time_coverage'] <= 1.
    assert full['fk_crosscheck_max_error'] < 1e-6
    data = np.genfromtxt(root / 'circle/tracking.csv', delimiter=',', names=True)
    radius = full['config']['radius']
    assert np.ptp(data['actual_x_m']) > 1.8 * radius
    assert np.ptp(data['actual_y_m']) > 1.8 * radius
    errors = np.sqrt(sum((data[f'actual_{axis}_m'] - data[f'ref_{axis}_m'])**2 for axis in 'xyz'))
    rmse = float(np.sqrt(np.mean(errors**2)))
    assert abs(rmse - full['metrics']['position_rmse_m']) < 1e-12
    assert rmse < 0.005 and full['metrics']['max_sample_gap_s'] <= 0.25
    results['circle'] = full
    if args.circle_only:
        (root / 'results.json').write_text(json.dumps({'passed': True, 'cases': results}, indent=2, ensure_ascii=False) + '\n')
        print(f'画圆、显示及演示节点关闭通过；全部子节点正常关闭={full["shutdown_clean"]}：{root}')
        return
    planned = launch_case(root, 'plan-only', ['plan_only:=true'], args.domain_id + 1)
    assert planned['status'] == 'planned_only' and planned['fraction'] == 1.
    assert not planned['execution_success'] and planned['samples'] == 0
    results['plan_only'] = planned
    results['blocked'] = blocked_case(root, args.domain_id + 2)
    for name, parameter in [('invalid-radius', 'radius:=-0.01'), ('reject-non-simulation', 'use_sim_time:=false')]:
        destination = root / name
        run(['ros2', 'run', 'franka_learning_demos', 'circle_demo', '--ros-args',
             '-p', 'use_sim_time:=true', '-p', 'keep_alive:=false', '-p', parameter,
             '-p', f'output_dir:={destination}'], root / f'{name}.log',
            dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id + 3), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'))
        result = json.loads((destination / 'summary.json').read_text())
        assert result['status'] == 'failed' and not result['execution_success'] and result['samples'] == 0
        results[name] = result
    (root / 'results.json').write_text(json.dumps({'passed': True, 'cases': results}, indent=2, ensure_ascii=False) + '\n')
    print(f'通过：画圆、仅规划、碰撞拒绝、无效半径、非仿真时间拒绝。结果：{root}')


if __name__ == '__main__':
    main()
