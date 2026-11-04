# Copyright 2026 multipanda_ros2 contributors
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = FindPackageShare('franka_description')
    # The URDF contains ros2_control metadata, but no controller manager is
    # started here. Only the independent libfranka reader connects to hardware.
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([description, 'robots', 'real', 'panda_arm.urdf.xacro']),
        ' hand:=false',
    ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Panda IP address'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='franka_control2', executable='franka_read_only_node',
            parameters=[{'robot_ip': LaunchConfiguration('robot_ip'), 'use_sim_time': False}],
            output='screen', on_exit=Shutdown(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                description, 'rviz', 'visualize_franka.rviz'])],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen', on_exit=Shutdown(),
        ),
    ])
