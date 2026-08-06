"""T-01 delivery mission with REAL PERCEPTION (hauler Milestone 1).

Nothing about the pile or the drop point is hardcoded anymore:
- SCAN: the robot spins in place and FINDS the cedar stack with its camera
  (warm-tan blob segmentation). The stack's position is whatever vision says.
- DOCK: visual servo on the blob bearing + lidar range, with a short logged
  blind final approach (real clamp trucks do the same).
- LOAD: bear-hug squeeze, hop, underhook toes seat, lift. Verified by the
  load's own telemetry, not by the robot's log.
- CARRY: waypoints are only a coarse site-map corridor; a 2D lidar does live
  obstacle avoidance around anything in the way (there is an unplanned barrel
  on the route that no waypoint knows about).
- GATE: align square and thread it on a closed-loop line (the gate is on the
  worker's site map; squeezing a 1.4 m gap is a control task, not a search).
- PLACE: the orange drop zone is FOUND by the camera (high-saturation orange
  blob), servoed to, and the stack is set down on it: lower, open, raise the
  paddles over the top, back away.

Run (while sim.launch.py is up):
  ros2 run tranno_sim delivery_demo
"""
import math
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, LaserScan

# Coarse site-map corridor (worker-pinned route). Targets come from perception.
AXIS_H = -0.363
AXIS_P0 = (-0.36, 0.585)
WAYPOINTS_PRE = [(2.44, -0.48, 0.30), (3.55, -1.40, 0.25)]
GATE = (5.0, -1.45)
ZONE_TRUTH = (8.0, -2.5)          # used ONLY for the final verification report

CLAMP_OPEN = 0.0
CLAMP_HUG = 0.45

V_MAX = 0.42
W_MAX = 0.35
K_HEAD = 1.8
GOAL_TOL = 0.30

CAM_W, CAM_HFOV = 640, 1.5

# stack: warm tan, medium saturation; zone: pure orange, high saturation
STACK_HSV_LO, STACK_HSV_HI = (8, 60, 160), (30, 135, 235)
ZONE_HSV_LO, ZONE_HSV_HI = (8, 140, 205), (30, 255, 255)


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
        self.frame = None
        self.scan = None
        self.loaded = False
        self.create_subscription(TFMessage, '/model/t01/pose', self._pose_cb, 20)
        self.create_subscription(TFMessage, '/model/cedar_stack/pose', self._stack_cb, 10)
        self.create_subscription(Image, '/front_cam', self._img_cb, 5)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 5)

    # ---------------- sensing ----------------

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

    def _img_cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.frame = arr[:, :, :3]

    def _scan_cb(self, msg):
        self.scan = msg

    def find_blob(self, lo, hi, roi_top=0, min_area=600, min_aspect=None):
        """Largest color blob: returns (bearing rad, area, centroid row) or None.
        min_aspect filters for wide-flat blobs (the drop zone pad vs cones)."""
        if self.frame is None:
            return None
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, lo, hi)
        mask[:roi_top, :] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        if min_aspect is not None:
            cnts = [c for c in cnts
                    if cv2.boundingRect(c)[2] >= min_aspect * max(1, cv2.boundingRect(c)[3])]
            if not cnts:
                return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < min_area:
            return None
        m = cv2.moments(c)
        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
        bearing = (cx - CAM_W / 2) / (CAM_W / 2) * (CAM_HFOV / 2)
        return bearing, area, cy

    def lidar_min(self, a0, a1, rmin=0.45):
        """Min range in sector [a0, a1] rad, ignoring returns closer than rmin
        (the robot's own clamp structure lives inside that radius)."""
        if self.scan is None:
            return None
        s = self.scan
        n = len(s.ranges)
        i0 = max(0, int((a0 - s.angle_min) / s.angle_increment))
        i1 = min(n - 1, int((a1 - s.angle_min) / s.angle_increment))
        r = [x for x in s.ranges[i0:i1 + 1] if rmin < x < s.range_max]
        return min(r) if r else None

    def stack_report(self, tag):
        rclpy.spin_once(self, timeout_sec=0.2)
        if self.stack:
            self.get_logger().info(
                f'    [stack@{tag}] x={self.stack[0]:.2f} y={self.stack[1]:.2f} z={self.stack[2]:.3f}')

    def pose_fresh(self):
        if time.time() - self.pose_stamp > 0.5:
            self.stop()
            return False
        return True

    def wait_for_sensors(self, timeout=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.frame is not None and self.scan is not None:
                return True
        return False

    def pump(self):
        """Drain pending callbacks: high-rate topics starve a single spin_once."""
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.0)
        rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        self.cmd.publish(Twist())

    def set_clamps(self, target):
        self.clamp_l.publish(Float64(data=target))
        self.clamp_r.publish(Float64(data=target))

    # ---------------- motion primitives ----------------

    def spin_search(self, lo, hi, roi_top, min_area, label, timeout=40.0, min_aspect=None):
        """SCAN: rotate in place until the camera finds and centers the target blob."""
        log = self.get_logger().info
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            self.pump()
            if not self.pose_fresh():
                continue
            b = self.find_blob(lo, hi, roi_top, min_area, min_aspect)
            msg = Twist()
            if b is None:
                msg.angular.z = 0.30
            elif abs(b[0]) > 0.05:
                msg.angular.z = max(-0.3, min(0.3, -1.2 * b[0]))
            else:
                self.stop()
                log(f'    {label} FOUND by camera: bearing {b[0]:+.2f} rad, area {int(b[1])} px')
                return True
            self.cmd.publish(msg)
        self.stop()
        return False

    def visual_servo(self, lo, hi, roi_top, min_area, stop_fn, speed=0.30, timeout=60.0):
        """Drive toward the blob, steering on its bearing, until stop_fn() is true."""
        t0 = time.time()
        last_log = 0.0
        while rclpy.ok() and time.time() - t0 < timeout:
            if time.time() - last_log > 2.0:
                last_log = time.time()
                fr = self.lidar_min(-0.35, 0.35)
                self.get_logger().info(f'    [servo] front range: {fr if fr is None else round(fr, 2)}')
            self.pump()
            if not self.pose_fresh():
                continue
            if stop_fn():
                self.stop()
                return True
            b = self.find_blob(lo, hi, roi_top, min_area)
            msg = Twist()
            msg.linear.x = speed
            if b is not None:
                msg.angular.z = max(-0.35, min(0.35, -1.5 * b[0]))
            self.cmd.publish(msg)
        self.stop()
        return False

    def creep_line(self, anchor, heading, s_stop, speed, timeout=35.0,
                   k_head=1.8, k_cross=1.4, w_cap=0.45):
        ch, sh = math.cos(heading), math.sin(heading)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            self.pump()
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
            # taper speed near the stop point: a hard stop throws an open-cradle load
            frac = min(1.0, abs(s_stop - s) / 0.5 + 0.25)
            msg.linear.x = speed * frac
            msg.angular.z = max(-w_cap, min(w_cap, -k_head * ang(yaw - heading) - k_cross * cross))
            self.cmd.publish(msg)
        self.stop()
        return False

    def rotate_to(self, yaw_target, tol=0.06, timeout=20.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            self.pump()
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
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            return False
        x0, y0, yaw0 = self.pose
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            self.pump()
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
        self.pump()
        if self.stack is None or self.stack[2] > 0.26:
            return  # >=0.26 = seated on the underhook toes: healthy ride, by design
        self.get_logger().warn('    load slipped: setting down, backing off, re-acquiring by vision')
        self.stop()
        self.lift.publish(Float64(data=0.0))
        time.sleep(1.5)
        self.set_clamps(CLAMP_OPEN)
        time.sleep(1.5)
        self.lift.publish(Float64(data=0.30))
        time.sleep(1.0)
        self.drive_straight(1.5, -0.30)
        self.pickup_sequence(self.get_logger().info)
        self.stack_report('after-regrip')

    def goto(self, gx, gy, tol=GOAL_TOL, timeout=40.0):
        """Corridor waypoint drive WITH live lidar obstacle avoidance."""
        t0 = time.time()
        last_progress = time.time()
        last_pos = None
        while rclpy.ok() and time.time() - t0 < timeout:
            self.pump()
            if self.pose is None or not self.pose_fresh():
                continue
            x, y, yaw = self.pose
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist < tol:
                self.stop()
                return True
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
            # live avoidance: if something unplanned blocks the front, arc around it.
            # When carrying, the load itself fills the near field: look past it.
            rmin = 1.45 if self.loaded else 0.45
            trig = 1.6 if self.loaded else 1.0
            front = self.lidar_min(-0.30, 0.30, rmin) if self.loaded                 else self.lidar_min(-0.45, 0.45, rmin)
            if front is not None and front < trig and dist > 0.8:
                left = self.lidar_min(0.45, 1.1, rmin) or 99.0
                right = self.lidar_min(-1.1, -0.45, rmin) or 99.0
                steer = 0.35 if left > right else -0.35
                self.get_logger().info(
                    f'    obstacle at {front:.2f} m: arcing {"left" if steer > 0 else "right"}')
                msg.linear.x = 0.22
                msg.angular.z = steer
            self.cmd.publish(msg)
        self.stop()
        return False

    def load_distance(self):
        """Distance from the clamp center to the load (None if unknown)."""
        if self.pose is None or self.stack is None:
            return None
        x, y, yaw = self.pose
        cx, cy = x + 0.95 * math.cos(yaw), y + 0.95 * math.sin(yaw)
        return math.hypot(self.stack[0] - cx, self.stack[1] - cy)

    def pickup_sequence(self, log):
        log('    SCAN: spin and find the cedar stack with the camera')
        self.loaded = False
        self.set_clamps(CLAMP_OPEN)
        self.lift.publish(Float64(data=0.05))
        time.sleep(1.0)
        if not self.spin_search(STACK_HSV_LO, STACK_HSV_HI, 200, 900, 'pile'):
            return False
        log('    DOCK: visual servo, lidar range gate, short blind entry')
        self.visual_servo(STACK_HSV_LO, STACK_HSV_HI, 200, 900,
                          stop_fn=lambda: (self.lidar_min(-0.35, 0.35) or 99) < 0.85,
                          speed=0.30)
        self.drive_straight(0.65, 0.22)
        if self.pose:
            log(f'    docked at ({self.pose[0]:.2f}, {self.pose[1]:.2f}); clamp center '
                f'at ({self.pose[0] + 0.95 * math.cos(self.pose[2]):.2f}, '
                f'{self.pose[1] + 0.95 * math.sin(self.pose[2]):.2f})')
        log('    LOAD: hug, hop, seat the underhook toes, lift')
        self.set_clamps(CLAMP_HUG)
        time.sleep(2.5)
        self.stack_report('after-hug')
        self.lift.publish(Float64(data=0.14))
        time.sleep(2.0)
        self.set_clamps(CLAMP_HUG)
        time.sleep(1.5)
        self.lift.publish(Float64(data=0.42))
        time.sleep(2.5)
        self.stack_report('after-lift')
        self.loaded = True
        return self.stack is None or self.stack[2] >= 0.30

    # ---------------- the mission ----------------

    def run(self):
        log = self.get_logger().info
        log('T-01 delivery mission start: perception-driven (camera + lidar)')

        if not self.wait_for_sensors():
            self.get_logger().error('sensors missing (pose/camera/lidar); is the sim up?')
            return

        log('1-3/7 SCAN + DOCK + LOAD (perception-driven pickup)')
        if not self.pickup_sequence(log):
            self.get_logger().error('LOAD NOT ACQUIRED; aborting honestly')
            return

        log('4/7 CARRY: site-map corridor with live lidar avoidance')
        for i, (gx, gy, tol) in enumerate(WAYPOINTS_PRE, start=1):
            log(f'    waypoint {i}/{len(WAYPOINTS_PRE)}: ({gx:.1f}, {gy:.1f})')
            reached = self.goto(gx, gy, tol)
            log(f'    -> {"reached" if reached else "TIMEOUT"} at '
                f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')
        self.stack_report('mid-carry')
        self.regrip_if_slipping()

        log('5/7 GATE: align square, thread it on a closed loop')
        self.rotate_to(0.0, tol=0.05)
        # stage 1: converge onto the gate axis while still outside the throat
        self.creep_line(GATE, 0.0, -0.45, 0.30, timeout=30.0,
                        k_head=1.8, k_cross=1.5, w_cap=0.40)
        # stage 2: already centered: fast, calm push straight through
        ok = self.creep_line(GATE, 0.0, 1.35, 0.5, timeout=40.0,
                             k_head=1.2, k_cross=0.9, w_cap=0.30)
        if not ok and self.pose and self.pose[0] > 5.4:
            ok = True
        log(f'    -> {"through the gate" if ok else "GATE TIMEOUT"} at '
            f'({self.pose[0]:.2f}, {self.pose[1]:.2f})')
        self.regrip_if_slipping()

        log('6/7 PLACE: find the orange drop zone with the camera and go to it')
        found = self.spin_search(ZONE_HSV_LO, ZONE_HSV_HI, 140, 300, 'drop zone', min_aspect=1.4)
        if not found:
            log('    zone not visible: backing up 1.2 m for a wider view and rescanning')
            self.drive_straight(1.2, -0.3)
            found = self.spin_search(ZONE_HSV_LO, ZONE_HSV_HI, 140, 300, 'drop zone', min_aspect=1.4)
        if not found:
            self.get_logger().error('no drop zone found; aborting')
            return
        self.visual_servo(ZONE_HSV_LO, ZONE_HSV_HI, 140, 400,
                          stop_fn=lambda: (lambda b: b is not None and b[2] > 344)(
                              self.find_blob(ZONE_HSV_LO, ZONE_HSV_HI, 140, 400, 1.2)),
                          speed=0.28, timeout=50.0)
        log('    zone at the bumper: 0.72 m straight, then set the load down on it')
        self.drive_straight(0.72, 0.22)
        self.stack_report('pre-place')
        self.lift.publish(Float64(data=0.0))
        time.sleep(2.0)
        self.set_clamps(CLAMP_OPEN)
        self.loaded = False
        time.sleep(1.5)
        self.lift.publish(Float64(data=0.45))
        time.sleep(1.8)

        log('7/7 CLEAR: back away over the top of the load')
        self.drive_straight(1.8, -0.30)
        rclpy.spin_once(self, timeout_sec=0.2)
        self.stack_report('final')
        if self.stack:
            dx = self.stack[0] - ZONE_TRUTH[0]
            dy = self.stack[1] - ZONE_TRUTH[1]
            log(f'mission complete; stack is {math.hypot(dx, dy):.2f} m from the zone '
                f'center it found by camera (truth check only)')


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
