"""Closed-loop delivery mission for T-01 (Phase B).

Waypoint-follows on /odom (P-controller on heading + distance), then deposits the
stack with the tilt-deck dump: detach -> tilt up -> stack slides off -> tilt back,
back away.

Run (while sim.launch.py is up):
  ros2 run tranno_sim delivery_demo
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64, Empty

# route: around the scrap obstacle, THROUGH the 1.15 m yard gate at x=5.0, over bump2,
# past the zone so the rear tilt-dump lands the stack ON the zone (it drops ~0.6 m behind)
WAYPOINTS = [(2.4, -0.5), (3.4, -1.35), (4.15, -1.45), (6.0, -1.45), (7.2, -2.3), (8.9, -2.6)]

V_MAX = 0.9          # m/s
W_MAX = 0.9          # rad/s
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
        self.detach = self.create_publisher(Empty, '/detach', 10)
        self.pose = None
        self.create_subscription(TFMessage, '/model/t01/pose', self._pose_cb, 20)

    def _pose_cb(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id == 't01':
                t = tr.transform
                self.pose = (t.translation.x, t.translation.y, yaw_from_quat(t.rotation))

    def wait_for_odom(self, timeout=15.0):
        t0 = time.time()
        while self.pose is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def stop(self):
        self.cmd.publish(Twist())

    def goto(self, gx, gy, timeout=40.0):
        """P-controller drive to (gx, gy) in world frame, with stuck recovery."""
        t0 = time.time()
        last_progress = time.time()
        last_pos = None
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue
            x, y, yaw = self.pose
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist < GOAL_TOL:
                self.stop()
                return True
            # stuck recovery: no progress for 6 s -> back up and retry
            if last_pos is None or math.hypot(x - last_pos[0], y - last_pos[1]) > 0.05:
                last_pos = (x, y)
                last_progress = time.time()
            elif time.time() - last_progress > 6.0:
                self.get_logger().warn('    no progress, backing up to retry')
                back = Twist()
                back.linear.x = -0.4
                for _ in range(30):
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

    def run(self):
        log = self.get_logger().info
        log('T-01 delivery mission start (closed-loop)')

        if not self.wait_for_odom():
            self.get_logger().error('no /model/t01/pose; is the sim + bridge running?')
            return

        log('1/6 secure load: lift 0.12, deck level')
        self.tilt.publish(Float64(data=0.0))
        self.lift.publish(Float64(data=0.12))
        time.sleep(2.0)

        for i, (gx, gy) in enumerate(WAYPOINTS, start=1):
            log(f'2/6 waypoint {i}/{len(WAYPOINTS)}: ({gx:.1f}, {gy:.1f})')
            reached = self.goto(gx, gy)
            log(f'    -> {"reached" if reached else "TIMEOUT"} at '
                f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')

        log('3/6 on zone: lift down')
        self.lift.publish(Float64(data=0.0))
        time.sleep(2.0)

        log('4/6 release payload')
        self.detach.publish(Empty())
        time.sleep(0.6)

        log('5/6 tilt-deck dump')
        self.tilt.publish(Float64(data=-0.55))
        time.sleep(3.0)
        self.tilt.publish(Float64(data=0.0))
        time.sleep(1.5)

        log('6/6 turn and clear the zone (never reverse into the dropped stack)')
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
                f'stack should rest on the drop zone')


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
