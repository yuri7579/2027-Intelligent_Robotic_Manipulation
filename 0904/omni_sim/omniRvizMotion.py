#!/usr/bin/env python3
"""
omniRvizMotion.py
=================
ROS 2 interface for the **master** robot motion.

* subscribes  ``/omni/joint_states``  (published by joint_state_publisher_gui)
* publishes   ``/omni/joint_states``  (optional - to drive the master model
  from a script, e.g. when replaying a recorded trajectory)

It is a thin python wrapper around those two topics so that a teleoperation
script never has to touch rclpy message plumbing directly::

    rclpy.init()
    master = OmniRvizMotion()
    q = master.get_current_joint(wait=True)
    print(master.get_ee_pose())
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:                                    # run as a module: python3 -m omni_sim.omniRvizMotion
    from .kinematics import omniVar
    from .kinematics.omniKinematics import OmniKinematics
except ImportError:                     # run as a script: python3 omni_sim/omniRvizMotion.py
    from omni_sim.kinematics import omniVar
    from omni_sim.kinematics.omniKinematics import OmniKinematics

NAMESPACE = '/omni'


class OmniRvizMotion(Node):
    """Python wrapper that reads / writes the master joint states."""

    def __init__(self, node_name='omni_rviz_motion', namespace=NAMESPACE, publish=True):
        """
        Args:
            node_name (str): ROS 2 node name.
            namespace (str): topic namespace of the master robot.
            publish (bool): also create a publisher on ``<ns>/joint_states``.
                Set to False while the GUI owns the topic to avoid fighting it.
        """
        super().__init__(node_name)

        self.namespace = namespace.rstrip('/')
        self.joint_names = list(omniVar.JOINT_NAMES)
        self.num_joints = len(self.joint_names)

        self._joint = None          # np.ndarray (6,) - latest master joint angles
        self._stamp = None          # rclpy.time.Time  - when it was received

        self.joint_state_topic = '{}/joint_states'.format(self.namespace)
        self._sub = self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_cb, 10)
        self._pub = self.create_publisher(
            JointState, self.joint_state_topic, 10) if publish else None

        self.get_logger().info(
            "OmniRvizMotion ready (subscribing '{}')".format(self.joint_state_topic))

    # ------------------------------------------------------------------ #
    # callbacks
    # ------------------------------------------------------------------ #
    def _joint_state_cb(self, msg):
        """Store the incoming joint state, reordered into JOINT_NAMES order."""
        q = self._reorder(msg)
        if q is not None:
            self._joint = q
            self._stamp = self.get_clock().now()

    def _reorder(self, msg):
        """Pick m1..m6 out of a JointState message by *name*, not by index.

        joint_state_publisher_gui does not guarantee any particular order, so
        matching by name is the only safe way.
        """
        if not msg.name:
            # No names given: fall back to positional order if the size fits.
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
        """Latest master joint angles [rad] as (6,) ndarray, or None."""
        return None if self._joint is None else self._joint.copy()

    def get_current_joint(self, wait=False, timeout=5.0):
        """Return the latest master joint angles.

        Args:
            wait (bool): block until the first message arrives.  Only use this
                when *this* node is not already being spun by an executor -
                it calls ``rclpy.spin_once(self)`` internally.
            timeout (float): give up after this many seconds and return None.
        """
        if wait and self._joint is None:
            deadline = time.time() + timeout
            while rclpy.ok() and self._joint is None and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
        return self.joint

    def is_ready(self):
        """True once at least one master joint state has been received."""
        return self._joint is not None

    def get_ee_pose(self, joint=None):
        """End-effector pose of the master (position [m], rotation matrix).

        Returns None when no joint state has been received yet.
        """
        q = self.joint if joint is None else np.asarray(joint, dtype=float)
        if q is None:
            return None
        return OmniKinematics.fk_pose(q)

    # ------------------------------------------------------------------ #
    # write
    # ------------------------------------------------------------------ #
    def set_joint(self, joint):
        """Publish a joint state so that the master model moves in RViz.

        Args:
            joint (array-like, shape (6,)): joint angles [rad].
        """
        if self._pub is None:
            raise RuntimeError('OmniRvizMotion was created with publish=False')
        q = omniVar.clip(joint)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in q]
        self._pub.publish(msg)
        return q


def main(args=None):
    """Small demo: print the master joint angles and end-effector position."""
    rclpy.init(args=args)
    node = OmniRvizMotion(publish=False)
    np.set_printoptions(precision=4, suppress=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.is_ready():
                p, _ = node.get_ee_pose()
                print('q = {}   p_ee = {}'.format(node.joint, p), end='\r', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
