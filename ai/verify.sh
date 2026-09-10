#!/usr/bin/env bash
# Verify the teleop stack against ROS's own tf, inside a ROS 2 environment.
#
#   ./docker/run.sh bash /workspace/ai/verify.sh        # via the Jazzy container
#   bash ai/verify.sh                                   # on a native ROS 2 machine
#
# What it checks:
#   1. both robot_state_publishers come up and expose the namespaced topics
#   2. frame_prefix is applied (omni/, omy/)
#   3. the teleop loop passes master joint values through to the slave
#   4. fk_pose() of both robots matches the tf that robot_state_publisher
#      publishes, over random joint configurations
# (no `set -u`: ROS 2 setup.bash reads unset variables)

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/0904"
cd "$WS"

cleanup() { kill $(jobs -p) 2>/dev/null; }
trap cleanup EXIT

echo "### 1) launching both state publishers (no GUI, no RViz)"
ros2 launch omni_sim/launch/omni_rviz.launch.py gui:=false rviz:=false >/tmp/omni.log 2>&1 &
ros2 launch omy_sim/launch/omy_rviz.launch.py  gui:=false rviz:=false >/tmp/omy.log  2>&1 &
sleep 6
ros2 topic list | grep -E "^/(omni|omy)/"

echo
echo "### 2) frame_prefix"
ros2 param get /omni_state_publisher frame_prefix
ros2 param get /omy_state_publisher  frame_prefix

echo
echo "### 3) teleop node"
python3 omy_teleop/omyTeleop.py --no-launch --log-period 0 >/tmp/teleop.log 2>&1 &
sleep 3

echo
echo "### 4) fk_pose() vs tf"
python3 - "$WS" <<'PY'
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

sys.path.insert(0, sys.argv[1])
from omni_sim.kinematics.omniKinematics import OmniKinematics
from omni_sim.kinematics import omniVar
from omy_sim.kinematics.omyKinematics import OmyKinematics

rclpy.init()
node = Node('verifier')
pub = node.create_publisher(JointState, '/omni/joint_states', 10)
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, node)


def spin(seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.02)


def publish(q):
    msg = JointState()
    # NOTE: a real timestamp is mandatory - robot_state_publisher drops
    # JointState messages whose header.stamp is zero (this is why
    # `ros2 topic pub` appears to do nothing).
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = list(omniVar.JOINT_NAMES)
    msg.position = [float(v) for v in q]
    pub.publish(msg)


def lookup(parent, child):
    t = buf.lookup_transform(parent, child, rclpy.time.Time()).transform.translation
    return np.array([t.x, t.y, t.z])


rng = np.random.default_rng(7)
lo, hi = omniVar.JOINT_LIMITS[:, 0], omniVar.JOINT_LIMITS[:, 1]
worst_master = worst_slave = 0.0

print('{:>3}  {:>22}  {:>22}'.format('#', 'master |tf-fk| [m]', 'slave |tf-fk| [m]'))
for k in range(8):
    q = rng.uniform(lo, hi)
    for _ in range(25):
        publish(q)
        spin(0.02)
    spin(0.5)
    e_m = np.abs(lookup('omni/base', 'omni/stylus') - OmniKinematics.fk_pose(q)[0]).max()
    # the teleop maps 1:1, so the slave sits at the same joint values
    e_s = np.abs(lookup('omy/world', 'omy/end_effector_link') - OmyKinematics.fk_pose(q)[0]).max()
    worst_master, worst_slave = max(worst_master, e_m), max(worst_slave, e_s)
    print('{:>3}  {:>22.3e}  {:>22.3e}'.format(k, e_m, e_s))

print()
print('worst master error: {:.3e} m'.format(worst_master))
print('worst slave  error: {:.3e} m'.format(worst_slave))
ok = max(worst_master, worst_slave) < 1e-6
print('RESULT:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
PY
