"""
Forward kinematics of the master device (Geomagic Touch / Phantom Omni).

Modified (Craig) DH convention, identical to the convention used in the
lab's ROS 1 code so that results can be compared 1:1.

    T_i-1,i = | cos(th)            -sin(th)            0           a        |
              | sin(th)cos(al)      cos(th)cos(al)    -sin(al)    -sin(al)d |
              | sin(th)sin(al)      cos(th)sin(al)     cos(al)     cos(al)d |
              | 0                   0                  0           1        |
"""

import numpy as np

from . import omniVar


class OmniKinematics:
    """Static helper class - no state, only kinematic maths."""

    @classmethod
    def DH_transform(cls, dhparams):
        """Homogeneous transform of every neighboring frame pair.

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
    def fk(cls, joints):
        """Forward kinematics.

        Args:
            joints (array-like, shape (6,)): joint angles [rad].

        Returns:
            (list[np.ndarray], list[np.ndarray]):
                Tbs - base->i transforms, ``Tbs[-1]`` is base->end-effector,
                Ts  - the individual link transforms [T_b1, T_12, ...].
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

    @classmethod
    def fk_pose(cls, joints):
        """End-effector pose of the master.

        Returns:
            (np.ndarray, np.ndarray): position (3,) [m] and rotation matrix (3, 3).
        """
        Tbs, _ = cls.fk(joints)
        Tbe = Tbs[-1]
        return Tbe[:3, 3], Tbe[:3, :3]


if __name__ == '__main__':
    # Quick sanity check without ROS: FK at the home posture.
    q = omniVar.HOME_JOINT
    p, Rm = OmniKinematics.fk_pose(q)
    np.set_printoptions(precision=4, suppress=True)
    print('q      =', q)
    print('p_ee   =', p)
    print('R_ee   =\n', Rm)
