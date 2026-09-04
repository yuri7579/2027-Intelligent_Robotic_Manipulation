"""
Robot-specific constants of the slave robot (ROBOTIS OMY-3M).

All values are taken straight out of ``description/urdf/omy_3m.urdf`` so the
kinematics can never drift away from the model shown in RViz.

Kinematic chain (every ``<origin rpy>`` in the URDF is zero, so each joint is a
pure translation followed by a rotation about its own axis)::

    world --fixed--> link0 --joint1(z)--> link1 --joint2(y)--> link2
          --joint3(y)--> link3 --joint4(y)--> link4 --joint5(z)--> link5
          --joint6(y)--> link6 --fixed--> end_effector_link
"""

import numpy as np

# ---------------------------------------------------------------------------
# Joint definition (order matters: it is the order used on /omy/joint_states)
# ---------------------------------------------------------------------------
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
NUM_JOINTS = len(JOINT_NAMES)

# Joint limits [rad] from the <limit> tags of the URDF (+/- 2*pi on every joint).
JOINT_LIMITS = np.tile(np.array([[-2 * np.pi, 2 * np.pi]]), (NUM_JOINTS, 1))

# q = 0 is the "candle" posture: the arm points straight up along +z.
HOME_JOINT = np.zeros(NUM_JOINTS)

# ---------------------------------------------------------------------------
# Link geometry [m] - <joint><origin xyz="..."> of the URDF
# ---------------------------------------------------------------------------
JOINT_ORIGINS = np.array([[0.0, 0.0000, 0.1715],    # link0 -> joint1
                          [0.0, -0.1215, 0.0000],   # joint1 -> joint2
                          [0.0, 0.0000, 0.2470],    # joint2 -> joint3
                          [0.0, 0.1215, 0.2195],    # joint3 -> joint4
                          [0.0, -0.1130, 0.0000],   # joint4 -> joint5
                          [0.0, 0.0000, 0.1155]])   # joint5 -> joint6

# <joint><axis xyz="..."> of the URDF
JOINT_AXES = np.array([[0.0, 0.0, 1.0],             # joint1 yaw
                       [0.0, 1.0, 0.0],             # joint2 pitch
                       [0.0, 1.0, 0.0],             # joint3 pitch
                       [0.0, 1.0, 0.0],             # joint4 pitch
                       [0.0, 0.0, 1.0],             # joint5 roll
                       [0.0, 1.0, 0.0]])            # joint6 pitch

# link6 -> end_effector_link (fixed joint "end_effector_joint")
EE_OFFSET = np.array([0.0, -0.103, 0.0])

# Base frame name and tip frame name inside the URDF (without the tf prefix).
BASE_LINK = 'link0'
EE_LINK = 'end_effector_link'


def clip(joints):
    """Clamp joint values into JOINT_LIMITS."""
    q = np.asarray(joints, dtype=float)
    return np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
