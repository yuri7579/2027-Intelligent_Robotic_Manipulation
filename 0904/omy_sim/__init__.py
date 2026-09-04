"""
omy_sim
=======
Simulation / visualization package for the **slave** robot
(ROBOTIS OMY-3M, 6-DOF manipulator).

Follows exactly the same structure as :mod:`omni_sim`:
    description/   robot model (URDF, meshes, joint/link information)
    launch/        ROS 2 launch files (robot_state_publisher, RViz)
    kinematics/    forward kinematics / Jacobian, joint and pose conversion
    omyRviz.py     launches the OMY visualization in RViz
    omyRvizMotion.py   ROS 2 python wrapper (motion wrapper) of the slave robot
"""

import os

PKG_NAME = 'omy_sim'
PKG_DIR = os.path.dirname(os.path.abspath(__file__))

NAMESPACE = '/omy'
FRAME_PREFIX = 'omy/'

__all__ = ['PKG_NAME', 'PKG_DIR', 'NAMESPACE', 'FRAME_PREFIX']
