#!/usr/bin/env python3
"""Save frames from an Image topic to PNGs: record_cam.py <topic> <outdir>"""
import sys, os, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2


class Rec(Node):
    def __init__(self, topic, outdir):
        super().__init__('rec_' + topic.strip('/').replace('/', '_'))
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self.n = 0
        self.sub = self.create_subscription(Image, topic, self.cb, 10)

    def cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8',):
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        cv2.imwrite(os.path.join(self.outdir, f"f_{self.n:06d}.jpg"), arr)
        self.n += 1


def main():
    topic, outdir = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = Rec(topic, outdir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(f"saved {node.n} frames to {outdir}")


if __name__ == '__main__':
    main()
