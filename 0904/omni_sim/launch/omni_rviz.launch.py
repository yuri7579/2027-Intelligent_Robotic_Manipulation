#!/usr/bin/env python3
"""
omni_rviz.launch.py
===================
Brings up the **master** (Geomagic Touch / Phantom Omni) visualization.

Nodes started here
    robot_state_publisher       /omni/joint_states  ->  /tf, /tf_static
    joint_state_publisher_gui   sliders             ->  /omni/joint_states
    rviz2                       /tf, /omni/robot_description  ->  3D view

Every topic lives under the ``/omni`` namespace and every tf frame is prefixed
with ``omni/`` so that the master and the slave can coexist on the same ROS 2
graph without any name collision.

Launch arguments
    gui:=true|false          start joint_state_publisher_gui   (default true)
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

# .../0904/omni_sim
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# .../0904  -> makes `import omni_sim` work without any colcon build / install
WS_DIR = os.path.dirname(PKG_DIR)
if WS_DIR not in sys.path:
    sys.path.insert(0, WS_DIR)

from omni_sim.description import URDF_PATH, load_urdf  # noqa: E402

NAMESPACE = '/omni'
FRAME_PREFIX = 'omni/'


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # URDF is read once, here, and passed to the nodes as a plain string
    # parameter - no colcon build and no ament index needed.
    robot_description = load_urdf()

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start joint_state_publisher_gui (slider window).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(PKG_DIR, 'launch', 'omni.rviz'),
            description='RViz configuration file.'),

        # /omni/joint_states -> /tf (+ /omni/robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='omni_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'frame_prefix': FRAME_PREFIX}],
            remappings=[('joint_states', NAMESPACE + '/joint_states'),
                        ('robot_description', NAMESPACE + '/robot_description')],
        ),

        # sliders -> /omni/joint_states   (this is how the master is "moved")
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='omni_joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(gui),
            arguments=[URDF_PATH],
            remappings=[('joint_states', NAMESPACE + '/joint_states'),
                        ('robot_description', NAMESPACE + '/robot_description')],
        ),

        # master RViz window
        Node(
            package='rviz2',
            executable='rviz2',
            name='omni_rviz',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', rviz_config],
        ),
    ])
