"""
Robot-specific constants of the master device (Geomagic Touch / Phantom Omni).

Every number below is taken from ``description/urdf/omni.urdf`` so that the
kinematics and the RViz model always agree with each other.

Kinematic chain::

    base --m1(-z)--> torso --m2(x)--> upper_arm --m3(x)--> lower_arm
         --m4(-y)--> wrist --m5(x)--> tip --m6(z)--> stylus

Unlike the OMY, this URDF has non-zero ``rpy`` on several joint origins, so a
joint transform is ``Trans(origin) * Rot_rpy(fixed) * Rot(axis, q)``.
"""

import numpy as np

# ---------------------------------------------------------------------------
# Joint definition (order matters: it is the order used on /omni/joint_states)
# ---------------------------------------------------------------------------
JOINT_NAMES = ['m1', 'm2', 'm3', 'm4', 'm5', 'm6']
NUM_JOINTS = len(JOINT_NAMES)

# Joint limits [rad], copied from the <limit> tags of the URDF.
JOINT_LIMITS = np.array([[-0.98, 0.98],     # m1  waist    (yaw)
                         [0.00, 1.75],      # m2  shoulder (pitch)
                         [-0.81, 1.25],     # m3  elbow    (pitch)
                         [-2.00, 2.00],     # m4  wrist    (roll)
                         [-1.50, 1.10],     # m5  wrist    (pitch)
                         [-2.58, 2.58]])    # m6  stylus   (yaw)

# A safe "ready" posture used when no master data has been received yet.
HOME_JOINT = np.array([0.0, 0.6, 0.0, 0.0, 0.0, 0.0])

# ---------------------------------------------------------------------------
# URDF link geometry - <joint><origin xyz rpy> and <joint><axis xyz>
# ---------------------------------------------------------------------------
JOINT_ORIGINS = np.array([[0.0000, 0.0000, 0.0900],     # base      -> m1
                          [-0.0075, 0.0000, 0.0350],    # torso     -> m2
                          [0.0075, 0.1340, 0.0000],     # upper_arm -> m3
                          [0.0000, 0.0800, 0.0000],     # lower_arm -> m4
                          [0.0000, 0.0525, 0.0000],     # wrist     -> m5
                          [0.0000, 0.0000, 0.0000]])    # tip       -> m6

JOINT_RPY = np.array([[0.0, 0.0, 3.14159265359],
                      [0.0, 0.0, 0.0],
                      [-1.5, 0.0, 0.0],                 # note: -1.5, not -pi/2
                      [0.0, 0.0, 0.0],
                      [1.570796, 0.0, 0.0],
                      [1.5707, -1.5707, 0.0]])

JOINT_AXES = np.array([[0.0, 0.0, -1.0],
                       [1.0, 0.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])

# Frame names inside the URDF (without the tf prefix).
BASE_LINK = 'base'
EE_LINK = 'stylus'

# ---------------------------------------------------------------------------
# Link lengths [m] of the classic modified-DH description (see dhparam below)
# ---------------------------------------------------------------------------
L1 = 0.09 + 0.035       # base  -> shoulder height
L2 = 0.1340             # upper arm length
L3 = 0.08 + 0.0525      # lower arm length


def dhparam(joints):
    """Modified-DH table [alpha, a, d, theta] of the Omni.

    .. warning::
        This table follows the frame convention of the **Geomagic/OpenHaptics
        device**, which is *not* the frame convention of the URDF links.  Its
        result is therefore rotated / offset with respect to the ``base`` and
        ``stylus`` frames that RViz shows.  Use :meth:`OmniKinematics.fk` when
        you need a pose that matches tf; use this one only to compare against
        the lab's existing (ROS 1) code.

    Args:
        joints (array-like, shape (6,)): joint angles [rad], order = JOINT_NAMES.

    Returns:
        np.ndarray, shape (6, 4): one [alpha, a, d, theta] row per joint.
    """
    q1, q2, q3, q4, q5, q6 = np.asarray(joints, dtype=float).T
    return np.array([[0.0, 0.0, L1, -np.pi - q1],
                     [-np.pi / 2, 0.0, 0.0, -q2],
                     [0.0, L2, 0.0, -q3],
                     [np.pi / 2, 0.0, -L3, q4],
                     [-np.pi / 2, 0.0, 0.0, -np.pi / 2 - q5],
                     [np.pi / 2, 0.0, 0.0, q6]])


def clip(joints):
    """Clamp joint values into JOINT_LIMITS."""
    q = np.asarray(joints, dtype=float)
    return np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
