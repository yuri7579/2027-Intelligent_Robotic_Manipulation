"""
omy_teleop
==========
Connects the **master** (omni_sim) and the **slave** (omy_sim).

    omyTeleop.py           entry point - launches both RViz windows and runs the loop
    mapping/               mapping utilities (master joint -> slave joint)
    teleop/                teleoperation modules (the ROS 2 node that runs the loop)

Week 2 scope: joint-to-joint mapping, one master joint to one slave joint,
values passed through as they are.
"""

import os
import sys

PKG_DIR = os.path.dirname(os.path.abspath(__file__))
# .../0904 - so that `import omni_sim` / `import omy_sim` work from anywhere
WS_DIR = os.path.dirname(PKG_DIR)
if WS_DIR not in sys.path:
    sys.path.insert(0, WS_DIR)

__all__ = ['PKG_DIR', 'WS_DIR']
