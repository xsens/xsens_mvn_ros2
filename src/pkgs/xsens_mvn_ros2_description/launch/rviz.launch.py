# Copyright (c) 2026, Xsens Technologies B.V.
# SPDX-License-Identifier: BSD-3-Clause
"""
Start RViz with one RobotModel display per robot_description topic.

The stock configuration shows a single model read from `/robot_description`.
A namespaced driver, or a scene with several actors, publishes its
descriptions elsewhere, so this launch rewrites the RobotModel display of the
given config into one display per topic before handing it to RViz.  A custom
config without a RobotModel display is passed through untouched.
"""
import copy
import os
import tempfile

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml

ROBOT_MODEL_CLASS = 'rviz_default_plugins/RobotModel'
DEFAULT_TOPIC = '/robot_description'


def parse_name_list(text):
    """Turn '[a, b]', 'a, b' or 'a' into ['a', 'b']; '' and '[]' give []."""
    value = yaml.safe_load(text) if text.strip() else []
    if not isinstance(value, list):
        value = str(value).split(',')
    return [str(v).strip() for v in value if str(v).strip()]


def display_name(topic):
    """'/actor_b/robot_description' -> 'RobotModel (actor_b)'."""
    namespace = topic.rsplit('/', 1)[0].strip('/')
    return f'RobotModel ({namespace})' if namespace else 'RobotModel'


def config_for_topics(config_path, topics):
    """Return an RViz config path whose RobotModel displays cover `topics`."""
    if not topics or topics == [DEFAULT_TOPIC]:
        return config_path

    with open(config_path) as f:
        config = yaml.safe_load(f)
    manager = config.get('Visualization Manager', {})
    displays = manager.get('Displays', [])
    templates = [d for d in displays if d.get('Class') == ROBOT_MODEL_CLASS]
    if not templates:
        return config_path

    robot_models = []
    for topic in topics:
        display = copy.deepcopy(templates[0])
        display['Name'] = display_name(topic)
        display.setdefault('Description Topic', {})['Value'] = topic
        robot_models.append(display)
    manager['Displays'] = [
        d for d in displays if d.get('Class') != ROBOT_MODEL_CLASS] + robot_models

    fd, path = tempfile.mkstemp(prefix='xsens_rviz_', suffix='.rviz')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(config, f)
    return path


def launch_setup(context):
    config_path = LaunchConfiguration('rviz_config_file').perform(context)
    topics = parse_name_list(
        LaunchConfiguration('robot_description_topics').perform(context))
    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='xsens_rviz',
            output='screen',
            arguments=['-d', config_for_topics(config_path, topics)],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('xsens_mvn_ros2_description'),
                'config', 'xsens_visualization.rviz']),
            description='RViz config to start from.'),
        DeclareLaunchArgument(
            'robot_description_topics', default_value=f'[{DEFAULT_TOPIC}]',
            description='One RobotModel display is created per topic in this list.'),
        OpaqueFunction(function=launch_setup),
    ])
