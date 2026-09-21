# Copyright (c) 2026, Xsens Technologies B.V.
# SPDX-License-Identifier: BSD-3-Clause
"""
Start the MVN stream node and, by default, configure and activate it.

Every node parameter a multi-suit setup needs is exposed as a launch argument
so a scene with several avatars can be brought up without editing the config
file.  `avatar_names` is a list; give it as `[actor_a, actor_b, prop]`.
"""
import launch
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import lifecycle_msgs.msg

import yaml


def parse_name_list(text):
    """Turn '[a, b]', 'a, b' or 'a' into ['a', 'b']; '' and '[]' give []."""
    value = yaml.safe_load(text) if text.strip() else []
    if not isinstance(value, list):
        value = str(value).split(',')
    return [str(v).strip() for v in value if str(v).strip()]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return IfCondition(LaunchConfiguration(name)).evaluate(context)

    # Typed explicitly: a bare string would be re-typed by YAML rules, so a
    # model_name such as "007" would otherwise reach the node as an integer.
    parameters = {
        'model_name': ParameterValue(arg('model_name'), value_type=str),
        'reference_frame': ParameterValue(arg('reference_frame'), value_type=str),
        'udp_port': int(arg('udp_port')),
        'avatar_id': int(arg('avatar_id')),
        'track_all_avatars': flag('track_all_avatars'),
    }
    # Only set avatar_names when given: an empty list is the node's default
    # anyway, and this side-steps writing an untyped empty array to the
    # generated parameter file.
    avatar_names = parse_name_list(arg('avatar_names'))
    if avatar_names:
        parameters['avatar_names'] = avatar_names

    stream_node = LifecycleNode(
        package='xsens_mvn_ros2_stream',
        executable='xsens_mvn_ros2_stream_node',
        name='xsens_mvn_ros2_stream_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('xsens_mvn_ros2_stream'),
                'config', 'xsens_stream_node.yaml']),
            parameters,
        ],
    )

    configure_event = EmitEvent(
        condition=IfCondition(LaunchConfiguration('auto_activate')),
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(stream_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_handler = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration('auto_activate')),
        event_handler=OnStateTransition(
            target_lifecycle_node=stream_node,
            goal_state='inactive',
            handle_once=True,
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            stream_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return [stream_node, activate_handler, configure_event]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace for the node and its topics.'),
        DeclareLaunchArgument(
            'auto_activate', default_value='true',
            description='Configure and activate the lifecycle node on startup.'),
        DeclareLaunchArgument(
            'model_name', default_value='skeleton',
            description='TF frame prefix of the primary avatar (skeleton -> skeleton_pelvis).'),
        DeclareLaunchArgument(
            'reference_frame', default_value='world',
            description='Root TF frame the skeletons are expressed in.'),
        DeclareLaunchArgument(
            'udp_port', default_value='9763',
            description='UDP port MVN streams to.'),
        DeclareLaunchArgument(
            'avatar_id', default_value='0',
            description='MVN avatar published on the unprefixed topics.'),
        DeclareLaunchArgument(
            'track_all_avatars', default_value='false',
            description='Publish every avatar MVN streams, not only avatar_id.'),
        DeclareLaunchArgument(
            'avatar_names', default_value='[]',
            description='TF prefix per avatar id, e.g. [actor_a, actor_b, prop].'),
        OpaqueFunction(function=launch_setup),
    ])
