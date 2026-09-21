# Copyright (c) 2026, Xsens Technologies B.V.
# SPDX-License-Identifier: BSD-3-Clause
"""
Bring up the MVN stream driver: stream node, URDF publisher(s) and RViz.

Single suit (the default) starts one of each.  For a multi-avatar scene pass
`track_all_avatars:=true avatar_names:=[actor_a, actor_b, prop]` and, for the
avatars that are tracked objects rather than bodies, `object_avatars:=[prop]`.
Each body then gets its own URDF publisher and RobotModel display, laid out
the same way the stream node lays out its topics: the primary avatar in the
launch namespace, every other body in a sub-namespace named after it.
"""
import launch
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import yaml


def parse_name_list(text):
    """Turn '[a, b]', 'a, b' or 'a' into ['a', 'b']; '' and '[]' give []."""
    value = yaml.safe_load(text) if text.strip() else []
    if not isinstance(value, list):
        value = str(value).split(',')
    return [str(v).strip() for v in value if str(v).strip()]


def join_namespace(*parts):
    """Join namespace parts, dropping empties: ('', 'a') -> 'a'."""
    return '/'.join(p.strip('/') for p in parts if p.strip('/'))


def include(package, launch_file, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=launch_arguments.items(),
    )


def body_avatars(namespace, model_name, avatar_id, track_all, names, objects):
    """Return (model_name, namespace) for every body that needs a URDF.

    Mirrors XsensStreamNode::avatarName so the URDF publisher looks for the
    frames the stream node actually broadcasts: an avatar_names entry wins,
    otherwise the primary avatar is model_name.  Only named avatars can be
    given a URDF publisher here, since the id of an unnamed one is not known
    until MVN streams it.
    """
    def name_of(index):
        if index < len(names) and names[index]:
            return names[index]
        return model_name if index == avatar_id else f'{model_name}_{index}'

    bodies = [(name_of(avatar_id), namespace)]
    if track_all:
        bodies += [
            (name, join_namespace(namespace, name))
            for index, name in enumerate(names) if index != avatar_id]
    return [(name, ns) for name, ns in bodies if name not in objects]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return IfCondition(LaunchConfiguration(name)).evaluate(context)

    bodies = body_avatars(
        namespace=arg('namespace'),
        model_name=arg('model_name'),
        avatar_id=int(arg('avatar_id')),
        track_all=flag('track_all_avatars'),
        names=parse_name_list(arg('avatar_names')),
        objects=set(parse_name_list(arg('object_avatars'))),
    )

    actions = [
        include(
            'xsens_mvn_ros2_stream', 'xsens_stream.launch.py',
            namespace=arg('namespace'),
            auto_activate=arg('auto_activate'),
            model_name=arg('model_name'),
            reference_frame=arg('reference_frame'),
            udp_port=arg('udp_port'),
            avatar_id=arg('avatar_id'),
            track_all_avatars=arg('track_all_avatars'),
            avatar_names=arg('avatar_names'),
        ),
    ]

    if flag('launch_description'):
        actions += [
            include(
                'xsens_mvn_ros2_description', 'description.launch.py',
                namespace=ns,
                auto_activate=arg('auto_activate'),
                model_name=name,
            ) for name, ns in bodies]

    if flag('launch_rviz'):
        topics = ['/' + join_namespace(ns, 'robot_description') for _, ns in bodies]
        actions.append(include(
            'xsens_mvn_ros2_description', 'rviz.launch.py',
            rviz_config_file=arg('rviz_config_file'),
            robot_description_topics=str(topics),
        ))

    return actions


def generate_launch_description():
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
            description='Start a URDF publisher per body avatar.'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('xsens_mvn_ros2_description'),
                'config', 'xsens_visualization.rviz']),
            description='RViz config; its RobotModel display is repeated per body avatar.'),
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
        DeclareLaunchArgument(
            'object_avatars', default_value='[]',
            description='Entries of avatar_names that are objects: no URDF or RobotModel.'),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', LaunchConfiguration('discovery_range')),
        OpaqueFunction(function=launch_setup),
    ])
