#!/usr/bin/env python3
"""
jointMapping.py
===============
Joint-to-joint mapping between the master (Omni, 6-DOF) and the slave
(OMY-3M, 6-DOF).

Both devices have six revolute joints, so the simplest possible mapping is a
one-to-one pass-through of the joint values::

    m1 -> joint1    m2 -> joint2    m3 -> joint3
    m4 -> joint4    m5 -> joint5    m6 -> joint6

The general form implemented below is::

    q_slave[i] = clip( sign[i] * scale[i] * (q_master[i] - master_offset[i])
                       + slave_offset[i] ,  slave joint limits )

With the Week 2 defaults (sign=+1, scale=1, offsets=0) this reduces to
``q_slave = q_master``, i.e. the joint values are used as they are.
The extra terms are already here so that Week 4 (motion mapping / scaling)
only has to change numbers, not code.

Note that this is a *joint-space* mapping, not a task-space one: the two arms
have completely different link lengths and joint axes, so the end-effector
poses will not match.  Matching the end-effector poses is the Week 4 topic
(``T_master_ee^master_base  =  T_slave_ee^slave_base``).
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from omni_sim.kinematics import omniVar     # noqa: E402
from omy_sim.kinematics import omyVar       # noqa: E402


class JointMapper:
    """Maps a master joint vector to a slave joint vector."""

    def __init__(self, scale=None, sign=None, master_offset=None, slave_offset=None,
                 master_names=None, slave_names=None,
                 master_limits=None, slave_limits=None, clip=True):
        """
        Args:
            scale (array-like (6,)): per-joint gain. Default 1 (no scaling).
            sign (array-like (6,)): per-joint direction, +1 or -1. Default +1.
            master_offset (array-like (6,)): master value considered "zero" [rad].
            slave_offset (array-like (6,)): slave value at the master zero [rad].
            master_names (list[str]): master joint order. Default m1..m6.
            slave_names (list[str]): slave joint order. Default joint1..joint6.
            master_limits (np.ndarray (6, 2)): master joint limits [rad].
            slave_limits (np.ndarray (6, 2)): slave joint limits [rad].
            clip (bool): clamp the result into the slave joint limits.
        """
        self.master_names = list(master_names or omniVar.JOINT_NAMES)
        self.slave_names = list(slave_names or omyVar.JOINT_NAMES)
        if len(self.master_names) != len(self.slave_names):
            raise ValueError('joint-to-joint mapping needs the same number of joints '
                             'on both sides ({} vs {})'.format(len(self.master_names),
                                                               len(self.slave_names)))
        self.num_joints = len(self.master_names)

        self.master_limits = np.array(
            omniVar.JOINT_LIMITS if master_limits is None else master_limits, dtype=float)
        self.slave_limits = np.array(
            omyVar.JOINT_LIMITS if slave_limits is None else slave_limits, dtype=float)

        ones = np.ones(self.num_joints)
        zeros = np.zeros(self.num_joints)
        self.scale = ones.copy() if scale is None else np.asarray(scale, dtype=float)
        self.sign = ones.copy() if sign is None else np.asarray(sign, dtype=float)
        self.master_offset = (zeros.copy() if master_offset is None
                              else np.asarray(master_offset, dtype=float))
        self.slave_offset = (zeros.copy() if slave_offset is None
                             else np.asarray(slave_offset, dtype=float))
        self.clip = clip

        for name, value in (('scale', self.scale), ('sign', self.sign),
                            ('master_offset', self.master_offset),
                            ('slave_offset', self.slave_offset)):
            if value.shape != (self.num_joints,):
                raise ValueError("'{}' must have shape ({},), got {}".format(
                    name, self.num_joints, value.shape))

    # ------------------------------------------------------------------ #
    def map(self, q_master):
        """Map master joint angles to slave joint angles.

        Args:
            q_master (array-like, shape (6,)): master joint angles [rad].

        Returns:
            np.ndarray, shape (6,): slave joint angles [rad].
        """
        q_m = np.asarray(q_master, dtype=float).reshape(-1)
        if q_m.shape[0] != self.num_joints:
            raise ValueError('expected {} master joints, got {}'.format(
                self.num_joints, q_m.shape[0]))

        q_s = self.sign * self.scale * (q_m - self.master_offset) + self.slave_offset
        if self.clip:
            q_s = np.clip(q_s, self.slave_limits[:, 0], self.slave_limits[:, 1])
        return q_s

    __call__ = map

    def describe(self):
        """Human readable summary of the current mapping."""
        lines = ['{:>8s} -> {:<8s} {:>7s} {:>7s} {:>10s} {:>10s}'.format(
            'master', 'slave', 'sign', 'scale', 'q_m offset', 'q_s offset')]
        for i in range(self.num_joints):
            lines.append('{:>8s} -> {:<8s} {:>7.1f} {:>7.3f} {:>10.3f} {:>10.3f}'.format(
                self.master_names[i], self.slave_names[i], self.sign[i], self.scale[i],
                self.master_offset[i], self.slave_offset[i]))
        return '\n'.join(lines)


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    mapper = JointMapper()
    print(mapper.describe())
    q_m = np.array([0.1, 0.5, -0.2, 0.3, -0.4, 1.0])
    print('\nq_master =', q_m)
    print('q_slave  =', mapper.map(q_m))
