#!/usr/bin/env python3
"""
omyTeleop.py
============
Week 2 assignment entry point: **master(Omni) -> slave(OMY-3M) joint-to-joint
teleoperation, visualized in RViz**.

Running this single file will

    1. launch the master RViz     (omni_sim/omniRviz.py  -> omni_rviz.launch.py)
    2. launch the slave  RViz     (omy_sim/omyRviz.py    -> omy_rviz.launch.py)
    3. import the motion wrappers (OmniRvizMotion / OmyRvizMotion)
    4. read the master joint states from  /omni/joint_states
    5. map the master joints to the slave joints (1:1, values as they are)
    6. send the mapped joint command to     /omy/joint_states
    7. visualize the teleoperation in real time

Usage::

    python3 omy_teleop/omyTeleop.py                 # everything at once
    python3 omy_teleop/omyTeleop.py --rate 200      # faster loop
    python3 omy_teleop/omyTeleop.py --no-launch     # RViz windows already open

Drag the sliders of the "Joint State Publisher" window: the OMY model in the
second RViz window follows the Omni model joint by joint.
"""

import argparse
import os
import sys
import time

import rclpy

# .../0904 - so that omni_sim / omy_sim / omy_teleop are importable from anywhere
WS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WS_DIR not in sys.path:
    sys.path.insert(0, WS_DIR)

from omni_sim.omniRviz import OmniRviz                    # noqa: E402
from omni_sim.omniRvizMotion import OmniRvizMotion        # noqa: E402
from omy_sim.omyRviz import OmyRviz                       # noqa: E402
from omy_sim.omyRvizMotion import OmyRvizMotion           # noqa: E402
from omy_teleop.mapping.jointMapping import JointMapper   # noqa: E402
from omy_teleop.teleop.jointTeleop import JointTeleop, spin_teleop  # noqa: E402


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Omni(master) -> OMY(slave) joint-to-joint teleoperation in RViz.')
    parser.add_argument('--rate', type=float, default=100.0,
                        help='teleoperation loop rate [Hz] (default: 100)')
    parser.add_argument('--no-launch', action='store_true',
                        help='do not start the RViz windows (attach to running ones)')
    parser.add_argument('--no-master-gui', action='store_true',
                        help='do not start the master joint_state_publisher_gui')
    parser.add_argument('--startup-wait', type=float, default=4.0,
                        help='seconds to wait for RViz to come up (default: 4)')
    parser.add_argument('--log-period', type=float, default=1.0,
                        help='console print period [s], 0 to disable (default: 1)')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # ------------------------------------------------------------------ #
    # 1) + 2)  bring up the two RViz windows
    # ------------------------------------------------------------------ #
    visualizers = []
    if not args.no_launch:
        visualizers = [OmniRviz(gui=not args.no_master_gui),   # master + sliders
                       OmyRviz(gui=False)]                     # slave, teleop-driven
        for viz in visualizers:
            viz.start(wait=0.0)
        print('[teleop] waiting {:.1f}s for RViz to come up ...'.format(args.startup_wait))
        time.sleep(args.startup_wait)

    # ------------------------------------------------------------------ #
    # 3) ~ 7)  the teleoperation loop
    # ------------------------------------------------------------------ #
    rclpy.init()
    master = OmniRvizMotion(publish=False)   # the GUI owns /omni/joint_states
    slave = OmyRvizMotion()
    mapper = JointMapper()                   # Week 2: 1:1, joint values as they are
    teleop = JointTeleop(master, slave, mapper,
                         rate=args.rate, log_period=args.log_period)

    try:
        spin_teleop(master, slave, teleop)
    except KeyboardInterrupt:
        pass
    finally:
        print('\n[teleop] shutting down ...')
        for node in (teleop, slave, master):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for viz in visualizers:
            viz.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
