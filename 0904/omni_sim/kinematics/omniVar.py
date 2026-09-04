"""
Robot-specific constants of the master device (Geomagic Touch / Phantom Omni).

Every number below is taken from ``description/urdf/omni.urdf`` so that the
kinematics and the RViz model always agree with each other.
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
# Link lengths [m] (modified DH convention)
# ---------------------------------------------------------------------------
L1 = 0.09 + 0.035       # base  -> shoulder height
L2 = 0.1340             # upper arm length
L3 = 0.08 + 0.0525      # lower arm length


def dhparam(joints):
    """Modified-DH table [alpha, a, d, theta] of the Omni for the given joints.

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
