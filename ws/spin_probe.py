import rclpy, numpy as np, cv2, time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

rclpy.init()
n = rclpy.create_node('spin_probe')
state = {}


def cb(m):
    state['f'] = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]


n.create_subscription(Image, '/front_cam', cb, 5)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
t0 = time.time()
best = 0.0
best_frame = None
while time.time() - t0 < 24:
    for _ in range(8):
        rclpy.spin_once(n, timeout_sec=0.0)
    rclpy.spin_once(n, timeout_sec=0.02)
    m = Twist()
    m.angular.z = 0.30
    pub.publish(m)
    if 'f' in state:
        hsv = cv2.cvtColor(state['f'], cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (8, 140, 205), (30, 255, 255))
        mask[:140, :] = 0
        a = int(mask.sum() / 255)
        if a > best:
            best = a
            best_frame = state['f'].copy()
pub.publish(Twist())
print('max zone-mask pixels over a full spin:', best)
if best_frame is not None:
    cv2.imwrite('/tmp/spin_best.png', cv2.cvtColor(best_frame, cv2.COLOR_RGB2BGR))
