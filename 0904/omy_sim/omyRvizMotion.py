#!/usr/bin/env python3
"""
omyRvizMotion.py
================
ROS 2 interface (motion wrapper) for the **slave** robot, OMY-3M.

* publishes  ``/omy/joint_states``  -> robot_state_publisher -> /tf -> RViz
* subscribes ``/omy/joint_states``  so the wrapper always knows the posture
  that is currently displayed

This is the object a teleoperation script talks to when it wants to command
the slave robot::

    rclpy.init()
    slave = OmyRvizMotion()
    slave.set_joint([0.0, 0.3, -0.5, 0.0, 0.2, 0.0])
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:                                    # run as a module: python3 -m omy_sim.omyRvizMotion
    from .kinematics import omyVar
    from .kinematics.omyKinematics import OmyKinematics
except ImportError:                     # run as a script: python3 omy_sim/omyRvizMotion.py
    from omy_sim.kinematics import omyVar
    from omy_sim.kinematics.omyKinematics import OmyKinematics

NAMESPACE = '/omy'


class OmyRvizMotion(Node):
    """Python wrapper that commands / reads the slave joint states."""

    def __init__(self, node_name='omy_rviz_motion', namespace=NAMESPACE):
        """
        Args:
            node_name (str): ROS 2 node name.
            namespace (str): topic namespace of the slave robot.
        """
        super().__init__(node_name)

        self.namespace = namespace.rstrip('/')
        self.joint_names = list(omyVar.JOINT_NAMES)
        self.num_joints = len(self.joint_names)

        self._joint = None          # np.ndarray (6,) - last state seen on the topic
        self._command = np.array(omyVar.HOME_JOINT, dtype=float)

        self.joint_state_topic = '{}/joint_states'.format(self.namespace)
        self._pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self._sub = self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_cb, 10)

        self.get_logger().info(
            "OmyRvizMotion ready (publishing '{}')".format(self.joint_state_topic))

    # ------------------------------------------------------------------ #
    # callbacks
    # ------------------------------------------------------------------ #
    def _joint_state_cb(self, msg):
        q = self._reorder(msg)
        if q is not None:
            self._joint = q

    def _reorder(self, msg):
        """Pick joint1..joint6 out of a JointState message by *name*."""
        if not msg.name:
            if len(msg.position) >= self.num_joints:
                return np.asarray(msg.position[:self.num_joints], dtype=float)
            return None
        lookup = dict(zip(msg.name, msg.position))
        try:
            return np.array([lookup[name] for name in self.joint_names], dtype=float)
        except KeyError:
            return None

    # ------------------------------------------------------------------ #
    # read
    # ------------------------------------------------------------------ #
    @property
    def joint(self):
        """Latest slave joint angles [rad] as (6,) ndarray, or None."""
        return None if self._joint is None else self._joint.copy()

    @property
    def command(self):
        """Last joint command that was published, (6,) ndarray."""
        return self._command.copy()

    def is_ready(self):
        """True once at least one slave joint state has been received."""
        return self._joint is not None

    def get_ee_pose(self, joint=None):
        """End-effector pose of the slave (position [m], rotation matrix)."""
        q = self.joint if joint is None else np.asarray(joint, dtype=float)
        if q is None:
            return None
        return OmyKinematics.fk_pose(q)

    # ------------------------------------------------------------------ #
    # write
    # ------------------------------------------------------------------ #
    def set_joint(self, joint):
        """Publish a joint command; the model moves in RViz.

        Args:
            joint (array-like, shape (6,)): joint angles [rad].

        Returns:
            np.ndarray: the (clipped) command that was actually published.
        """
        q = omyVar.clip(joint)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in q]
        self._pub.publish(msg)
        self._command = q
        return q

    def go_home(self):
        """Publish the home posture (arm straight up)."""
        return self.set_joint(omyVar.HOME_JOINT)


def main(args=None):
    """Small demo: sweep joint1 back and forth so the slave model moves."""
    rclpy.init(args=args)
    node = OmyRvizMotion()
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            q = np.array(omyVar.HOME_JOINT, dtype=float)
            q[0] = 0.8 * np.sin(0.5 * (time.time() - t0))
            node.set_joint(q)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
