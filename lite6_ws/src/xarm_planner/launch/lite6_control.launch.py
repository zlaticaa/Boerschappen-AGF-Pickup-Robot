#!/usr/bin/env python3
import json
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='lite')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default='6')
    robot_type = LaunchConfiguration('robot_type', default='lite')

    node_executable = 'lite6_control'
    node_parameters = {}

    # robot planner launch
    # xarm_planner/launch/_robot_planner.launch.py
    robot_planner_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', '_robot_planner.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': 'false',
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'node_executable': node_executable,
            'node_parameters': json.dumps(node_parameters)
        }.items(),
    )

    return [
        robot_planner_node_launch
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
