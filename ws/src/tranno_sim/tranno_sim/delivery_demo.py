"""Closed-loop delivery mission for T-01: MECHANISM loading, no arm, no teleport.

The stack sits on dunnage stringers. The robot docks, slides its fork tines into
the channels, lifts, carries the 60 kg stack through the yard gate on rough
ground, then raises the mast, tips the tines forward into a ramp and lets the
stack slide off onto the drop zone. Load and unload are the same mechanism
reversed. Navigation is closed-loop on the true model pose.

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

# The pickup station is ROTATED to face the gate: dock, insert, backout and the run
# to the gate all share one axis (heading -0.363 rad), so the loaded robot never has
# to pivot hard. Waypoints follow that line, thread the gate, then bend gently to the zone.
AXIS_H = -0.363
AXIS_P0 = (-0.36, 0.585)
WAYPOINTS = [(1.3, -1.35, 0.30), (4.25, -1.45, 0.15), (5.75, -1.45, 0.25), (7.2, -2.3, 0.30), (8.9, -2.6, 0.30)]

V_MAX = 0.5          # m/s (gentle: the load rides on open forks)
W_MAX = 0.35         # rad/s (skid-steer chatter throws open loads)
K_HEAD = 1.8         # heading P gain
GOAL_TOL = 0.30      # m


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class DeliveryDemo(Node):
    def __init__(self):
        super().__init__('delivery_demo')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift = self.create_publisher(Float64, '/lift_cmd', 10)
        self.tilt = self.create_publisher(Float64, '/tilt_cmd', 10)
        self.pose = None
        self.pose_stamp = 0.0
        self.create_subscription(TFMessage, '/model/t01/pose', self._pose_cb, 20)

    def _pose_cb(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id == 't01':
                t = tr.transform
                self.pose = (t.translation.x, t.translation.y, yaw_from_quat(t.rotation))
                self.pose_stamp = time.time()

    def pose_fresh(self):
        """Never drive blind: if the pose stream stalls, stop and wait."""
        if time.time() - self.pose_stamp > 0.5:
            self.stop()
            return False
        return True

    def wait_for_odom(self, timeout=15.0):
        t0 = time.time()
        while self.pose is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def stop(self):
        self.cmd.publish(Twist())

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
            # stuck recovery: no progress for 6 s -> back up and retry
            # (pure rotation IS progress: rotate-in-place must not trigger this)
            if (last_pos is None
                    or math.hypot(x - last_pos[0], y - last_pos[1]) > 0.05
                    or abs(math.atan2(math.sin(yaw - last_pos[2]),
                                      math.cos(yaw - last_pos[2]))) > 0.15):
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
            err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
            msg = Twist()
            msg.angular.z = max(-W_MAX, min(W_MAX, K_HEAD * err))
            # rotate in place when badly off-heading (prevents orbiting the goal
            # and wedging into walls); otherwise scale speed by heading and distance
            if abs(err) > 1.0:
                msg.linear.x = 0.0
            else:
                msg.linear.x = max(0.08, V_MAX * math.cos(err) * min(1.0, dist / 1.2))
            self.cmd.publish(msg)
        self.stop()
        return False

    def creep_axis(self, s_stop, speed, timeout=35.0):
        """Straight drive along the pickup axis (AXIS_H through AXIS_P0), holding
        heading and cross-track. s = signed distance along the axis from AXIS_P0.
        Positive speed: insert; negative: back out of the dunnage."""
        ch, sh = math.cos(AXIS_H), math.sin(AXIS_H)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None or not self.pose_fresh():
                continue
            x, y, yaw = self.pose
            dx, dy = x - AXIS_P0[0], y - AXIS_P0[1]
            s = dx * ch + dy * sh
            cross = -dx * sh + dy * ch
            if (speed > 0 and s >= s_stop) or (speed < 0 and s <= s_stop):
                self.stop()
                return True
            herr = math.atan2(math.sin(yaw - AXIS_H), math.cos(yaw - AXIS_H))
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = max(-0.3, min(0.3, -1.5 * herr - 0.8 * cross))
            self.cmd.publish(msg)
        self.stop()
        return False

    def creep_insert(self, s_stop, speed=0.45, timeout=25.0):
        return self.creep_axis(s_stop, speed, timeout)

    def run(self):
        log = self.get_logger().info
        log('T-01 delivery mission start (mechanism load, closed-loop)')

        if not self.wait_for_odom():
            self.get_logger().error('no /model/t01/pose; is the sim + bridge running?')
            return

        log('1/7 dock: unbury the tines, level, then entry height')
        self.lift.publish(Float64(data=0.30))
        self.tilt.publish(Float64(data=0.0))
        time.sleep(2.0)
        self.lift.publish(Float64(data=0.02))
        time.sleep(1.5)

        log('2/7 insert tines into the dunnage channels')
        ok = self.creep_insert(0.90)
        log(f'    -> {"inserted" if ok else "INSERT TIMEOUT"} at ({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        log('3/7 lift the stack just clear, rack the mast back, carry LOW')
        self.lift.publish(Float64(data=0.35))
        time.sleep(2.0)
        self.tilt.publish(Float64(data=-0.18))
        time.sleep(1.0)

        log('3b/7 back straight out of the dunnage before any turn')
        ok = self.creep_axis(-0.60, -0.45, timeout=35.0)
        self.lift.publish(Float64(data=0.15))
        time.sleep(1.0)
        log(f'    -> {"clear of the dunnage" if ok else "BACKOUT TIMEOUT"} at ({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        for i, (gx, gy, tol) in enumerate(WAYPOINTS, start=1):
            log(f'4/7 waypoint {i}/{len(WAYPOINTS)}: ({gx:.1f}, {gy:.1f})')
            reached = self.goto(gx, gy, tol)
            log(f'    -> {"reached" if reached else "TIMEOUT"} at '
                f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        log('5/7 on zone: level the forks, raise mast for the ramp dump')
        self.tilt.publish(Float64(data=0.0))
        time.sleep(1.0)
        self.lift.publish(Float64(data=0.42))
        time.sleep(2.0)

        log('6/7 tip the tines: ramp the stack down onto the zone')
        self.tilt.publish(Float64(data=0.65))
        time.sleep(3.5)
        self.tilt.publish(Float64(data=0.0))
        time.sleep(1.0)
        self.lift.publish(Float64(data=0.10))
        time.sleep(1.0)

        log('7/7 turn and clear the zone (never reverse into the dropped stack)')
        msg = Twist()
        msg.angular.z = 0.9
        t0 = time.time()
        while time.time() - t0 < 1.9:
            self.cmd.publish(msg)
            time.sleep(0.05)
        msg = Twist()
        msg.linear.x = 0.5
        t0 = time.time()
        while time.time() - t0 < 2.6:
            self.cmd.publish(msg)
            time.sleep(0.05)
        self.stop()

        rclpy.spin_once(self, timeout_sec=0.2)
        if self.pose:
            log(f'mission complete at ({self.pose[0]:.2f}, {self.pose[1]:.2f}); '
                f'stack should rest on the drop zone')  # verify with gz model -m cedar_stack -p


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
