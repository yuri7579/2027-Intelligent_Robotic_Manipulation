#!/usr/bin/env python3
"""
omy_rviz.launch.py
==================
Brings up the **slave** (ROBOTIS OMY-3M) visualization.

Nodes started here
    robot_state_publisher       /omy/joint_states  ->  /tf, /tf_static
    joint_state_publisher_gui   sliders            ->  /omy/joint_states  (optional)
    rviz2                       /tf, /omy/robot_description  ->  3D view

During teleoperation the sliders are *not* used: ``omy_teleop`` is what
publishes ``/omy/joint_states``.  That is why ``gui`` defaults to false here
(the master launch file defaults it to true).

Launch arguments
    gui:=true|false          start joint_state_publisher_gui   (default false)
    rviz:=true|false         start RViz                        (default true)
    rviz_config:=<path>      RViz configuration file
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# .../0904/omy_sim
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# .../0904  -> makes `import omy_sim` work without any colcon build / install
WS_DIR = os.path.dirname(PKG_DIR)
if WS_DIR not in sys.path:
    sys.path.insert(0, WS_DIR)

from omy_sim.description import URDF_PATH, load_urdf  # noqa: E402

NAMESPACE = '/omy'
FRAME_PREFIX = 'omy/'


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_description = load_urdf()

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Start joint_state_publisher_gui (only for standalone tests).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(PKG_DIR, 'launch', 'omy.rviz'),
            description='RViz configuration file.'),

        # /omy/joint_states -> /tf (+ /omy/robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='omy_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'frame_prefix': FRAME_PREFIX}],
            remappings=[('joint_states', NAMESPACE + '/joint_states'),
                        ('robot_description', NAMESPACE + '/robot_description')],
        ),

        # optional sliders, for testing the slave model on its own
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='omy_joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(gui),
            arguments=[URDF_PATH],
            remappings=[('joint_states', NAMESPACE + '/joint_states'),
                        ('robot_description', NAMESPACE + '/robot_description')],
        ),

        # slave RViz window
        Node(
            package='rviz2',
            executable='rviz2',
            name='omy_rviz',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', rviz_config],
        ),
    ])
