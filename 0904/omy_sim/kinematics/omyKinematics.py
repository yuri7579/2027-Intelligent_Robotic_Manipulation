"""
Forward kinematics / Jacobian of the slave robot (ROBOTIS OMY-3M).

The OMY URDF has ``rpy="0 0 0"`` on every joint origin, so the transform from
one joint frame to the next is simply::

    T_i-1,i(q_i) = Trans(origin_i) * Rot(axis_i, q_i)

which is built directly from the URDF numbers in :mod:`omyVar`.  Nothing is
hand-derived here, therefore the kinematics can never disagree with what
robot_state_publisher shows in RViz.
"""

import numpy as np

from . import omyVar


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


class OmyKinematics:
    """Static helper class - no state, only kinematic maths."""

    @classmethod
    def joint_transforms(cls, joints):
        """Transform of every neighboring joint frame pair.

        Args:
            joints (array-like, shape (6,)): joint angles [rad].

        Returns:
            list[np.ndarray]: [T_01, T_12, ...], each (4, 4).
        """
        q = np.asarray(joints, dtype=float)
        return [transform(origin, rot_axis(axis, qi))
                for origin, axis, qi in zip(omyVar.JOINT_ORIGINS, omyVar.JOINT_AXES, q)]

    @classmethod
    def fk(cls, joints):
        """Forward kinematics.

        Args:
            joints (array-like, shape (6,)): joint angles [rad].

        Returns:
            (list[np.ndarray], np.ndarray):
                Tbs - base(link0)->joint_i transforms, one per joint,
                Tbe - base(link0)->end_effector_link transform (4, 4).
        """
        Tbi = np.eye(4)
        Tbs = []
        for T in cls.joint_transforms(joints):
            Tbi = Tbi.dot(T)
            Tbs.append(Tbi)
        Tbe = Tbs[-1].dot(transform(omyVar.EE_OFFSET, np.eye(3)))
        return Tbs, Tbe

    @classmethod
    def fk_pose(cls, joints):
        """End-effector pose (position [m], rotation matrix) w.r.t. ``link0``."""
        _, Tbe = cls.fk(joints)
        return Tbe[:3, 3], Tbe[:3, :3]

    @classmethod
    def jacobian(cls, joints):
        """Geometric Jacobian of the end-effector w.r.t. ``link0``.

        Returns:
            np.ndarray, shape (6, 6): rows 0-2 linear, rows 3-5 angular.
        """
        Tbs, Tbe = cls.fk(joints)
        p_ee = Tbe[:3, 3]

        J = np.zeros((6, omyVar.NUM_JOINTS))
        for i, (Tbi, axis) in enumerate(zip(Tbs, omyVar.JOINT_AXES)):
            z_i = Tbi[:3, :3].dot(axis)     # joint axis expressed in the base frame
            p_i = Tbi[:3, 3]
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i
        return J


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    q = omyVar.HOME_JOINT
    p, Rm = OmyKinematics.fk_pose(q)
    print('q     =', q)
    print('p_ee  =', p)
    print('R_ee  =\n', Rm)
    print('J     =\n', OmyKinematics.jacobian(q))
