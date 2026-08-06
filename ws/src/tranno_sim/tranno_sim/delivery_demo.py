"""Closed-loop delivery mission for T-01: BEAR-HUG clamp loading, no arm, no forks,
no pallets, no attach cheats.

The 60 kg stack sits flat on the ground. The robot drives up so the stack is between
its two clamp paddles, hugs it (force-limited squeeze), lifts, carries it through the
yard gate on rough ground, then PLACES it centered on the drop zone: lower, open,
back away. Load and unload are the same mechanism reversed. Navigation is closed-loop
on the true model pose, with a stale-pose fuse (never drive blind).

Run (while sim.launch.py is up):
  ros2 run tranno_sim delivery_demo
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64

# The pickup axis points straight at the yard gate, so the loaded robot never has to
# pivot hard. The stack sits ON the axis; waypoints continue along it, thread the
# 1.4 m gate, then bend gently toward the drop zone.
AXIS_H = -0.363
AXIS_P0 = (-0.36, 0.585)
STACK_S = 2.0            # stack center, distance along the axis from AXIS_P0
CLAMP_REACH = 0.95       # clamp center sits this far ahead of the robot center
ZONE = (8.0, -2.5)       # drop zone center: the PLACE target for the stack

# (x, y, tolerance): tighter tolerance at the gate alignment point
WAYPOINTS_PRE = [(2.44, -0.48, 0.30), (3.55, -1.40, 0.25)]   # to the gate approach
GATE = (5.0, -1.45)                                          # gate center; transit heading = 0
WAYPOINTS_POST = []                                          # post-gate is deterministic

CLAMP_OPEN = 0.0
CLAMP_HUG = 0.45         # position target past contact; joint effort cap limits force

V_MAX = 0.5              # m/s (gentle with a hugged load)
W_MAX = 0.35             # rad/s (skid-steer chatter is rough on cargo)
K_HEAD = 1.8
GOAL_TOL = 0.30


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def ang(a):
    return math.atan2(math.sin(a), math.cos(a))


class DeliveryDemo(Node):
    def __init__(self):
        super().__init__('delivery_demo')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift = self.create_publisher(Float64, '/lift_cmd', 10)
        self.clamp_l = self.create_publisher(Float64, '/clamp_l_cmd', 10)
        self.clamp_r = self.create_publisher(Float64, '/clamp_r_cmd', 10)
        self.pose = None
        self.pose_stamp = 0.0
        self.stack = None
        self.create_subscription(TFMessage, '/model/t01/pose', self._pose_cb, 20)
        self.create_subscription(TFMessage, '/model/cedar_stack/pose', self._stack_cb, 10)

    def _pose_cb(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id == 't01':
                t = tr.transform
                self.pose = (t.translation.x, t.translation.y, yaw_from_quat(t.rotation))
                self.pose_stamp = time.time()

    def _stack_cb(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id == 'cedar_stack':
                t = tr.transform
                self.stack = (t.translation.x, t.translation.y, t.translation.z)

    def stack_report(self, tag):
        rclpy.spin_once(self, timeout_sec=0.2)
        if self.stack:
            self.get_logger().info(
                f'    [stack@{tag}] x={self.stack[0]:.2f} y={self.stack[1]:.2f} z={self.stack[2]:.3f}')

    def pose_fresh(self):
        """Never drive blind: if the pose stream stalls, stop and wait."""
        if time.time() - self.pose_stamp > 0.5:
            self.stop()
            return False
        return True

    def wait_for_pose(self, timeout=15.0):
        t0 = time.time()
        while self.pose is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def stop(self):
        self.cmd.publish(Twist())

    def set_clamps(self, target):
        self.clamp_l.publish(Float64(data=target))
        self.clamp_r.publish(Float64(data=target))

    # ---------------- motion primitives ----------------

    def creep_line(self, anchor, heading, s_stop, speed, timeout=35.0):
        """Closed-loop straight drive along a line (anchor + heading), holding
        heading and cross-track. s = signed distance along the line from anchor."""
        ch, sh = math.cos(heading), math.sin(heading)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None or not self.pose_fresh():
                continue
            x, y, yaw = self.pose
            dx, dy = x - anchor[0], y - anchor[1]
            s = dx * ch + dy * sh
            cross = -dx * sh + dy * ch
            if (speed > 0 and s >= s_stop) or (speed < 0 and s <= s_stop):
                self.stop()
                return True
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = max(-0.45, min(0.45, -1.8 * ang(yaw - heading) - 1.4 * cross))
            self.cmd.publish(msg)
        self.stop()
        return False

    def creep_axis(self, s_stop, speed, timeout=35.0):
        return self.creep_line(AXIS_P0, AXIS_H, s_stop, speed, timeout)

    def rotate_to(self, yaw_target, tol=0.06, timeout=20.0):
        """Gentle rotate in place to a heading."""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None or not self.pose_fresh():
                continue
            err = ang(yaw_target - self.pose[2])
            if abs(err) < tol:
                self.stop()
                return True
            msg = Twist()
            msg.angular.z = max(-W_MAX, min(W_MAX, K_HEAD * err))
            self.cmd.publish(msg)
        self.stop()
        return False

    def drive_straight(self, distance, speed, timeout=25.0):
        """Drive straight (sign of speed = direction) holding the current heading."""
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            return False
        x0, y0, yaw0 = self.pose
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None or not self.pose_fresh():
                continue
            x, y, yaw = self.pose
            if math.hypot(x - x0, y - y0) >= distance:
                self.stop()
                return True
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = max(-0.3, min(0.3, -1.5 * ang(yaw - yaw0)))
            self.cmd.publish(msg)
        self.stop()
        return False

    def regrip_if_slipping(self):
        """If the load has sagged in the jaws, stop and re-grip (lower, squeeze, lift).
        The real machine will do exactly this off its grip encoders."""
        rclpy.spin_once(self, timeout_sec=0.05)
        if self.stack is None or self.stack[2] > 0.35:
            return
        self.get_logger().warn('    load slipping in the jaws: stopping to re-grip')
        self.stop()
        self.lift.publish(Float64(data=0.0))
        time.sleep(2.0)
        self.set_clamps(CLAMP_HUG)
        time.sleep(1.0)
        self.lift.publish(Float64(data=0.42))
        time.sleep(2.0)
        self.stack_report('after-regrip')

    def goto(self, gx, gy, tol=GOAL_TOL, timeout=40.0):
        """P-controller drive to (gx, gy) in world frame, with stuck recovery."""
        t0 = time.time()
        last_progress = time.time()
        last_pos = None
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None or not self.pose_fresh():
                continue
            x, y, yaw = self.pose
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist < tol:
                self.stop()
                return True
            # stuck recovery: no progress (position OR heading) for 9 s -> back up, retry
            if (last_pos is None
                    or math.hypot(x - last_pos[0], y - last_pos[1]) > 0.05
                    or abs(ang(yaw - last_pos[2])) > 0.15):
                last_pos = (x, y, yaw)
                last_progress = time.time()
            elif time.time() - last_progress > 9.0:
                self.get_logger().warn('    no progress, backing up to retry')
                back = Twist()
                back.linear.x = -0.18
                for _ in range(24):
                    self.cmd.publish(back)
                    time.sleep(0.05)
                self.stop()
                last_progress = time.time()
                last_pos = None
                continue
            heading = math.atan2(dy, dx)
            err = ang(heading - yaw)
            msg = Twist()
            msg.angular.z = max(-W_MAX, min(W_MAX, K_HEAD * err))
            if abs(err) > 1.0:
                msg.linear.x = 0.0
            else:
                msg.linear.x = max(0.08, V_MAX * math.cos(err) * min(1.0, dist / 1.2))
            self.cmd.publish(msg)
        self.stop()
        return False

    # ---------------- the mission ----------------

    def run(self):
        log = self.get_logger().info
        log('T-01 delivery mission start (bear-hug clamp, closed-loop)')

        if not self.wait_for_pose():
            self.get_logger().error('no /model/t01/pose; is the sim + bridge running?')
            return

        log('1/6 dock: clamps open, straddle the stack')
        self.set_clamps(CLAMP_OPEN)
        self.lift.publish(Float64(data=0.05))
        time.sleep(1.5)
        ok = self.creep_axis(STACK_S - CLAMP_REACH, 0.35)
        log(f'    -> {"straddling the stack" if ok else "DOCK TIMEOUT"} at '
            f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        log('2/6 hug: close the clamps (force-limited squeeze)')
        self.set_clamps(CLAMP_HUG)
        time.sleep(2.5)

        self.stack_report('after-hug')
        log('3/6 lift a hand-width, seat the underhook toes, then lift clear')
        self.lift.publish(Float64(data=0.14))
        time.sleep(2.0)
        self.set_clamps(CLAMP_HUG)
        time.sleep(1.5)
        self.lift.publish(Float64(data=0.42))
        time.sleep(2.5)
        self.stack_report('after-lift')

        for i, (gx, gy, tol) in enumerate(WAYPOINTS_PRE, start=1):
            log(f'4/6 waypoint {i}/{len(WAYPOINTS_PRE)}: ({gx:.1f}, {gy:.1f})')
            reached = self.goto(gx, gy, tol)
            log(f'    -> {"reached" if reached else "TIMEOUT"} at '
                f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        self.stack_report('mid-carry')
        self.regrip_if_slipping()
        log('4/6 gate: align square, thread it dead straight (deterministic transit)')
        self.rotate_to(0.0, tol=0.05)
        ok = self.creep_line(GATE, 0.0, 1.60, 0.32, timeout=80.0)
        log(f'    -> {"through the gate" if ok else "GATE TIMEOUT"} at '
            f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        self.regrip_if_slipping()
        for i, (gx, gy, tol) in enumerate(WAYPOINTS_POST, start=1):
            log(f'4/6 waypoint {i}/{len(WAYPOINTS_POST)} (post-gate): ({gx:.1f}, {gy:.1f})')
            reached = self.goto(gx, gy, tol)
            log(f'    -> {"reached" if reached else "TIMEOUT"} at '
                f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        log('5/6 place: face the zone, advance until the load is centered on it')
        rclpy.spin_once(self, timeout_sec=0.1)
        x, y, _ = self.pose
        self.rotate_to(math.atan2(ZONE[1] - y, ZONE[0] - x))
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 40.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None or not self.pose_fresh():
                continue
            x, y, yaw = self.pose
            if math.hypot(ZONE[0] - x, ZONE[1] - y) <= CLAMP_REACH:
                self.stop()
                break
            msg = Twist()
            msg.linear.x = 0.22
            msg.angular.z = max(-0.25, min(0.25, K_HEAD * ang(
                math.atan2(ZONE[1] - y, ZONE[0] - x) - yaw)))
            self.cmd.publish(msg)
        self.stop()
        self.stack_report('pre-place')
        log('    setting it down gently')
        self.lift.publish(Float64(data=0.0))
        time.sleep(2.0)
        self.set_clamps(CLAMP_OPEN)
        time.sleep(1.5)
        log('6/6 back away clear over the top (the load stays exactly where placed)')
        self.drive_straight(1.8, -0.30)
        self.lift.publish(Float64(data=0.10))

        rclpy.spin_once(self, timeout_sec=0.2)
        self.stack_report('final')
        if self.pose:
            log(f'mission complete at ({self.pose[0]:.2f}, {self.pose[1]:.2f}); '
                f'stack z should be ~0.18 and x,y near ({ZONE[0]:.1f}, {ZONE[1]:.1f})')


def main():
    rclpy.init()
    node = DeliveryDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
