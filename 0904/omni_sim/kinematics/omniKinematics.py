"""
Forward kinematics of the master device (Geomagic Touch / Phantom Omni).

Two descriptions live here:

* :meth:`OmniKinematics.fk` - built directly from the URDF joint origins,
  fixed rpy rotations and axes.  Its output matches the ``omni/...`` tf frames
  that robot_state_publisher publishes, i.e. what RViz draws.  **Use this one.**
* :meth:`OmniKinematics.fk_dh` - the modified (Craig) DH description used by
  the lab's ROS 1 code.  It uses the device's own frame convention, so its
  pose does *not* coincide with the URDF ``base`` / ``stylus`` frames.  Kept
  for comparison with the existing code only.
"""

import numpy as np

from . import omniVar


def rpy_to_rotation(roll, pitch, yaw):
    """URDF fixed-axis rpy (R = Rz(yaw) @ Ry(pitch) @ Rx(roll))."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_axis(axis, theta):
    """Rotation matrix of ``theta`` [rad] about a unit ``axis`` (Rodrigues)."""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * K.dot(K)


def transform(translation, rotation):
    """Assemble a 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


class OmniKinematics:
    """Static helper class - no state, only kinematic maths."""

    # ------------------------------------------------------------------ #
    # URDF-consistent kinematics (matches tf / RViz)
    # ------------------------------------------------------------------ #
    @classmethod
    def joint_transforms(cls, joints):
        """Transform of every neighboring link frame pair.

        ``T_i-1,i(q) = Trans(origin) * Rot_rpy(fixed) * Rot(axis, q)``

        Returns:
            list[np.ndarray]: [T_base,torso, T_torso,upper_arm, ...], each (4, 4).
        """
        q = np.asarray(joints, dtype=float)
        Ts = []
        for origin, rpy, axis, qi in zip(omniVar.JOINT_ORIGINS, omniVar.JOINT_RPY,
                                         omniVar.JOINT_AXES, q):
            fixed = transform(origin, rpy_to_rotation(*rpy))
            Ts.append(fixed.dot(transform(np.zeros(3), rot_axis(axis, qi))))
        return Ts

    @classmethod
    def fk(cls, joints):
        """Forward kinematics in the URDF frames.

        Args:
            joints (array-like, shape (6,)): joint angles [rad].

        Returns:
            (list[np.ndarray], np.ndarray):
                Tbs - base->link_i transforms, one per joint,
                Tbe - base->stylus transform (4, 4). ``Tbs[-1] is Tbe``.
        """
        Tbi = np.eye(4)
        Tbs = []
        for T in cls.joint_transforms(joints):
            Tbi = Tbi.dot(T)
            Tbs.append(Tbi)
        return Tbs, Tbs[-1]

    @classmethod
    def fk_pose(cls, joints):
        """End-effector (stylus) pose w.r.t. ``base``: position [m], rotation (3, 3)."""
        _, Tbe = cls.fk(joints)
        return Tbe[:3, 3], Tbe[:3, :3]

    @classmethod
    def jacobian(cls, joints):
        """Geometric Jacobian of the stylus w.r.t. ``base``.

        Returns:
            np.ndarray, shape (6, 6): rows 0-2 linear, rows 3-5 angular.
        """
        Tbs, Tbe = cls.fk(joints)
        p_ee = Tbe[:3, 3]

        J = np.zeros((6, omniVar.NUM_JOINTS))
        T_prev = np.eye(4)
        for i, axis in enumerate(omniVar.JOINT_AXES):
            # axis of joint i lives in the frame *after* its fixed rpy rotation
            R_fixed = T_prev[:3, :3].dot(rpy_to_rotation(*omniVar.JOINT_RPY[i]))
            z_i = R_fixed.dot(axis)
            p_i = Tbs[i][:3, 3]
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i
            T_prev = Tbs[i]
        return J

    # ------------------------------------------------------------------ #
    # Modified-DH kinematics of the lab's ROS 1 code (device frames)
    # ------------------------------------------------------------------ #
    @classmethod
    def DH_transform(cls, dhparams):
        """Homogeneous transform of every neighboring frame pair (modified DH).

        Args:
            dhparams (np.ndarray, shape (n, 4)): rows of [alpha, a, d, theta].

        Returns:
            list[np.ndarray]: [T_b1, T_12, T_23, ...], each (4, 4).
        """
        return [np.array([[np.cos(theta), -np.sin(theta), 0.0, a],
                          [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha),
                           -np.sin(alpha), -np.sin(alpha) * d],
                          [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha),
                           np.cos(alpha), np.cos(alpha) * d],
                          [0.0, 0.0, 0.0, 1.0]])
                for [alpha, a, d, theta] in dhparams]

    @classmethod
    def fk_dh(cls, joints):
        """Forward kinematics in the **device** (modified DH) frames.

        .. warning::
            Not the same frames as the URDF / tf. See :func:`omniVar.dhparam`.
        """
        Ts = cls.DH_transform(omniVar.dhparam(joints))

        # A plain for-loop is intentionally used here: np.linalg.multi_dot is
        # much slower for this many small matrices.
        Tbi = np.eye(4)
        Tbs = []
        for T in Ts:
            Tbi = Tbi.dot(T)
            Tbs.append(Tbi)
        return Tbs, Ts


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    q = omniVar.HOME_JOINT
    p, Rm = OmniKinematics.fk_pose(q)
    print('q          =', q)
    print('p_stylus   =', p, '  (URDF frames, matches tf)')
    print('R_stylus   =\n', Rm)
    print('p_dh       =', OmniKinematics.fk_dh(q)[0][-1][:3, 3], '  (device frames)')
