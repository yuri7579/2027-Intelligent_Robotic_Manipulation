"""
omni_sim
========
Simulation / visualization package for the **master** device
(3D Systems Geomagic Touch, a.k.a. Phantom Omni, 6-DOF).

Layout (see Week 2 slides, "Software Structure for This Class")
    description/   robot model (URDF, meshes, joint/link information)
    launch/        ROS 2 launch files (robot_state_publisher, joint_state_publisher_gui, RViz)
    kinematics/    forward kinematics, joint/pose conversion
    omniRviz.py    launches the Omni visualization in RViz
    omniRvizMotion.py  ROS 2 python wrapper to read/write the Omni joint states
"""

import os

PKG_NAME = 'omni_sim'
PKG_DIR = os.path.dirname(os.path.abspath(__file__))

# ROS 2 namespace used by every omni_sim node.
NAMESPACE = '/omni'
# tf frames are published as "omni/<link name>" so that master and slave
# can share the global /tf tree without any frame-name collision.
FRAME_PREFIX = 'omni/'

__all__ = ['PKG_NAME', 'PKG_DIR', 'NAMESPACE', 'FRAME_PREFIX']
