#!/usr/bin/env python3
"""
jointTeleop.py
==============
The teleoperation node: reads the master joint states, maps them, and sends the
result to the slave, at a fixed rate.

    /omni/joint_states  --(OmniRvizMotion)-->  JointMapper  --(OmyRvizMotion)-->  /omy/joint_states

The node itself owns no robot logic at all: reading and writing is done by the
motion wrappers of ``omni_sim`` / ``omy_sim``, and the master->slave relation
lives in ``omy_teleop.mapping``.
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from omy_sim.kinematics import omyVar                    # noqa: E402
from omy_teleop.mapping.jointMapping import JointMapper  # noqa: E402


class JointTeleop(Node):
    """Periodically maps master joints to slave joints."""

    def __init__(self, master, slave, mapper=None, rate=100.0, log_period=1.0,
                 node_name='omy_joint_teleop'):
        """
        Args:
            master (omni_sim.omniRvizMotion.OmniRvizMotion): master motion wrapper.
            slave (omy_sim.omyRvizMotion.OmyRvizMotion): slave motion wrapper.
            mapper (JointMapper): mapping to apply. Default = 1:1 pass-through.
            rate (float): teleoperation loop rate [Hz].
            log_period (float): console print period [s]. 0 disables printing.
        """
        super().__init__(node_name)

        self.master = master
        self.slave = slave
        self.mapper = mapper if mapper is not None else JointMapper()
        self.rate = float(rate)
        self.log_period = float(log_period)

        self._master_seen = False
        self._last_log = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self.rate, self._loop)

        self.get_logger().info('joint-to-joint teleoperation @ {:.0f} Hz\n{}'.format(
            self.rate, self.mapper.describe()))
        self.get_logger().info('waiting for the master on {} ...'.format(
            self.master.joint_state_topic))

    # ------------------------------------------------------------------ #
    def _loop(self):
        """One teleoperation step. Called by the timer at ``rate`` Hz."""
        q_master = self.master.joint

        if q_master is None:
            # Master not alive yet: hold the slave at its home posture so the
            # model is already visible in RViz.
            self.slave.set_joint(omyVar.HOME_JOINT)
            return

        if not self._master_seen:
            self._master_seen = True
            self.get_logger().info('master connected - teleoperation running')

        q_slave = self.mapper.map(q_master)
        self.slave.set_joint(q_slave)
        self._log(q_master, q_slave)

    def _log(self, q_master, q_slave):
        """Throttled console output of the current master / slave joints."""
        if self.log_period <= 0.0:
            return
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds * 1e-9 < self.log_period:
            return
        self._last_log = now
        with np.printoptions(precision=3, suppress=True, floatmode='fixed'):
            self.get_logger().info('q_master {}  ->  q_slave {}'.format(
                np.asarray(q_master), np.asarray(q_slave)))


def spin_teleop(master, slave, teleop):
    """Spin the three nodes (master wrapper, slave wrapper, teleop) together.

    A single executor is enough: the callbacks are tiny and must stay ordered.
    """
    executor = SingleThreadedExecutor()
    for node in (master, slave, teleop):
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in (master, slave, teleop):
            executor.remove_node(node)


def main(args=None):
    """Run only the teleoperation node (RViz windows must already be up)."""
    sys.path.insert(0, os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from omni_sim.omniRvizMotion import OmniRvizMotion
    from omy_sim.omyRvizMotion import OmyRvizMotion

    rclpy.init(args=args)
    master = OmniRvizMotion(publish=False)
    slave = OmyRvizMotion()
    teleop = JointTeleop(master, slave)
    try:
        spin_teleop(master, slave, teleop)
    finally:
        for node in (teleop, slave, master):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
