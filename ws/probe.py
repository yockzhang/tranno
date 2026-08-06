import rclpy, numpy as np, cv2, time
from sensor_msgs.msg import Image

rclpy.init()
n = rclpy.create_node('probe')
state = {}


def cb(m):
    state['f'] = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]


n.create_subscription(Image, '/front_cam', cb, 5)
t0 = time.time()
while time.time() - t0 < 10 and 'f' not in state:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'f' not in state:
    print('NO FRAME in 10 s')
else:
    f = state['f']
    hsv = cv2.cvtColor(f, cv2.COLOR_RGB2HSV)
    cv2.imwrite('/tmp/probe_rgb.png', cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
    for label, lo, hi, roi in [('stack', (5, 80, 100), (30, 215, 235), 200),
                               ('zone', (8, 200, 160), (30, 255, 255), 140)]:
        mask = cv2.inRange(hsv, lo, hi)
        mask[:roi, :] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = sorted([cv2.contourArea(c) for c in cnts], reverse=True)[:4]
        print(label, 'blob areas:', areas)
        cv2.imwrite(f'/tmp/probe_{label}.png', mask)
    # HSV stats of the central-bottom region (where the stack should be at spawn)
    r = hsv[240:340, 200:440]
    print('center-bottom H/S/V medians:',
          float(np.median(r[:, :, 0])), float(np.median(r[:, :, 1])), float(np.median(r[:, :, 2])))
