"""Start an isolated, simulation-only Panda circle experiment."""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def start(context):
    def value(name):
        return context.launch_configurations[name]

    gui = value('gui').lower() == 'true'
    keep_alive = value('keep_alive').lower() == 'true'
    share = Path(get_package_share_directory('franka_learning_demos'))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('franka_moveit_config')) /
                                          'launch/sim_moveit.launch.py')),
        launch_arguments={'unpause': 'true', 'headless': str(not gui).lower(),
                          'no_render': str(not gui).lower(), 'use_rviz': 'false', 'db': 'false'}.items())
    demo = Node(
        package='franka_learning_demos', executable='circle_demo', output='screen',
        parameters=[{
            'use_sim_time': True, 'radius': float(value('radius')),
            'waypoints': int(value('waypoints')), 'velocity_scale': float(value('velocity_scale')),
            'acceleration_scale': float(value('acceleration_scale')),
            'output_dir': value('output_dir'), 'keep_alive': keep_alive,
            'plan_only': value('plan_only').lower() == 'true',
        }])
    actions = [RegisterEventHandler(OnProcessExit(
        target_action=demo, on_exit=[EmitEvent(event=Shutdown(reason='Circle demo finished'))])),
        simulation, demo]
    if gui:
        actions.append(Node(
            package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/circle.rviz')],
            parameters=[{'use_sim_time': True}], output='log'))
    return actions


def generate_launch_description():
    arguments = [
        ('gui', 'true', 'Open MuJoCo and RViz windows'),
        ('keep_alive', 'true', 'Keep the result visible until Ctrl+C; false for batch runs'),
        ('plan_only', 'false', 'Compute and validate the path without executing it'),
        ('radius', '0.03', 'Circle radius in metres, 0.005 to 0.08'),
        ('waypoints', '80', 'Number of circle segments, 20 to 720'),
        ('velocity_scale', '0.1', 'Joint velocity scaling, 0.01 to 0.3'),
        ('acceleration_scale', '0.1', 'Joint acceleration scaling, 0.01 to 0.3'),
        ('domain_id', '181', 'Dedicated ROS domain for this simulation session'),
        ('output_dir', str(Path('log/panda-circle') / datetime.now().strftime('%Y%m%d-%H%M%S-%f')),
         'New output directory; an existing directory is never overwritten'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, default, description in arguments] + [
            OpaqueFunction(function=lambda context: [
                SetEnvironmentVariable('ROS_DOMAIN_ID', context.launch_configurations['domain_id']),
                SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
            ]),
            OpaqueFunction(function=start),
        ])
