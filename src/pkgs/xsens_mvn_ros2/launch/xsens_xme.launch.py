# Copyright (c) 2026, Xsens Technologies B.V.
# SPDX-License-Identifier: BSD-3-Clause
"""Bring up the XME (direct hardware) driver: XME node, URDF publisher and RViz."""
import launch
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition=None, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package), 'launch', launch_file])),
        condition=condition,
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    auto_activate = LaunchConfiguration('auto_activate')
    model_name = LaunchConfiguration('model_name')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace for every node, topic and service.'),
        DeclareLaunchArgument(
            'auto_activate', default_value='true',
            description='Configure and activate all lifecycle nodes on startup.'),
        DeclareLaunchArgument(
            'discovery_range', default_value='LOCALHOST',
            description='ROS_AUTOMATIC_DISCOVERY_RANGE for the launched nodes.'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Start RViz.'),
        DeclareLaunchArgument(
            'launch_description', default_value='true',
            description='Start the URDF publisher.'),
        DeclareLaunchArgument(
            'model_name', default_value='skeleton',
            description='TF frame prefix (skeleton -> skeleton_pelvis).'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('xsens_mvn_ros2_description'),
                'config', 'xsens_visualization.rviz']),
            description='RViz config to start from.'),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', LaunchConfiguration('discovery_range')),

        include(
            'xsens_mvn_ros2_xme', 'xsens_xme.launch.py',
            namespace=namespace, auto_activate=auto_activate, model_name=model_name),
        include(
            'xsens_mvn_ros2_description', 'description.launch.py',
            condition=IfCondition(LaunchConfiguration('launch_description')),
            namespace=namespace, auto_activate=auto_activate, model_name=model_name),
        # The URDF publisher's topic is relative, so it follows the namespace;
        # point the RobotModel display at wherever it ended up.
        include(
            'xsens_mvn_ros2_description', 'rviz.launch.py',
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            rviz_config_file=LaunchConfiguration('rviz_config_file'),
            robot_description_topics=PathJoinSubstitution(
                ['/', namespace, 'robot_description'])),
    ])
