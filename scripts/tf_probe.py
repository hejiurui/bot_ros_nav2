
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFProbe(Node):
    def __init__(self, use_sim_time):
        super().__init__('tf_probe', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=use_sim_time)])
        self.frames = {}

        self.create_subscription(TFMessage, '/tf', self._cb, 100)
        self.create_subscription(TFMessage, '/tf_static', self._cb,
                                 rclpy.qos.QoSProfile(depth=100,
                                                      durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def _cb(self, msg):
        for t in msg.transforms:
            self.frames[t.child_frame_id] = t

def main(argv):
    pairs = argv or ['map->odom', 'odom->base_footprint',
                     'base_footprint->base_link', 'base_link->scanner_link',
                     'scanner_link->scanner_frame']
    use_sim = '--system-time' not in pairs
    pairs = [p for p in pairs if not p.startswith('--')]

    rclpy.init()
    node = TFProbe(use_sim_time=use_sim)

    t0 = time.monotonic()
    while time.monotonic() - t0 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('[tf_probe] 时钟=%s，已收到 %d 个帧的变换'
          % ('sim' if use_sim else 'system', len(node.frames)))
    rc = 0
    for pair in pairs:
        parent, child = [s.strip() for s in pair.split('->')]
        tf = node.frames.get(child)
        if tf is None:
            print('  MISSING  %s：%s 从未出现在 /tf 里' % (pair, child))
            rc = 1
            continue
        got_parent = tf.header.frame_id == parent
        tr = tf.transform.translation
        rot = tf.transform.rotation
        print('  %-6s  %s -> %-14s t=(%+.3f, %+.3f, %+.3f) q=(%+.3f,%+.3f,%+.3f,%+.3f) stamp=%.2f'
              % ('OK' if got_parent else 'WRONG', tf.header.frame_id, child,
                 tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w,
                 tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9))
        if not got_parent:
            print('           期望父帧 %s，实际 %s —— 谁在发布这条 TF？' % (parent, tf.header.frame_id))
            rc = 1
    if rc == 0:
        print('[tf_probe] TF 链路完整，每条边只有一个父帧（无重复发布者）')
    rclpy.shutdown() if rclpy.ok() else None
    return rc

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
