#!/usr/bin/env python3
"""Fuse the 2D lidar and the 4 ultrasonic sensors into one 360 deg LaserScan.

  /scan                    lidar, 360 deg, 0.15-12 m, scan plane 0.28 m above ground
  /ultrasonic/front_left   30 deg cone, 0.03-4 m, 0.12 m above ground (sees low obstacles)
  /ultrasonic/front_right
  /ultrasonic/rear_left
  /ultrasonic/rear_right
        |
        v   every ray end point -> base_ground (TF from the URDF), then binned by angle
  /scan_merged             one scan in base_ground, nearest return per angle bin

Overlap handling: where sensors look at the same direction (lidar + ultrasonics, or the two
front/rear ultrasonics with each other) the nearest valid return in a bin wins, so an
obstacle seen by any sensor is kept. "No echo" readings (range >= range_max, inf, nan) are
dropped instead of being treated as obstacles. Ultrasonic data older than max_age is ignored.
The output is published at the lidar rate, stamped with the lidar time.
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('ultrasonic_topics', ['/ultrasonic/front_left',
                                                     '/ultrasonic/front_right',
                                                     '/ultrasonic/rear_left',
                                                     '/ultrasonic/rear_right'])
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('output_frame', 'base_ground')
        self.declare_parameter('angle_increment_deg', 0.5)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('max_age', 0.3)        # s, ultrasonic data older than this is skipped
        self.declare_parameter('use_ultrasonics', True)

        self.frame = self.get_parameter('output_frame').value
        self.inc = math.radians(self.get_parameter('angle_increment_deg').value)
        self.n_bins = int(round(2 * math.pi / self.inc))
        self.rmin = self.get_parameter('range_min').value
        self.rmax = self.get_parameter('range_max').value
        self.max_age = Duration(seconds=self.get_parameter('max_age').value)
        self.use_us = self.get_parameter('use_ultrasonics').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sensor_tf = {}      # frame_id -> (R 3x3, t 3) sensor -> output frame (fixed joints)
        self.ray_cache = {}      # (frame, n, angle_min, inc) -> unit ray directions in output frame
        self.latest_us = {}      # topic -> LaserScan

        self.pub = self.create_publisher(LaserScan, self.get_parameter('output_topic').value, 10)
        self.create_subscription(LaserScan, self.get_parameter('lidar_topic').value,
                                 self.on_lidar, qos_profile_sensor_data)
        for topic in self.get_parameter('ultrasonic_topics').value:
            self.create_subscription(LaserScan, topic,
                                     lambda m, t=topic: self.latest_us.__setitem__(t, m),
                                     qos_profile_sensor_data)
        self.count = 0
        self.get_logger().info(
            f'merging {self.get_parameter("lidar_topic").value} + '
            f'{len(self.get_parameter("ultrasonic_topics").value) if self.use_us else 0} '
            f'ultrasonics -> {self.get_parameter("output_topic").value} '
            f'({self.n_bins} bins, frame {self.frame})')

    # sensor frames are fixed joints of the URDF, so their transform is looked up once
    def lookup(self, frame):
        if frame not in self.sensor_tf:
            try:
                tf = self.tf_buffer.lookup_transform(self.frame, frame, Time())
            except TransformException as e:
                self.get_logger().warn(f'waiting for TF {self.frame} <- {frame}: {e}',
                                       throttle_duration_sec=5.0)
                return None
            t = tf.transform.translation
            self.sensor_tf[frame] = (quat_to_rot(tf.transform.rotation), np.array([t.x, t.y, t.z]))
        return self.sensor_tf[frame]

    def points(self, scan):
        """Valid ray end points of one scan, as x, y arrays in the output frame."""
        T = self.lookup(scan.header.frame_id)
        if T is None:
            return None
        R, t = T
        r = np.asarray(scan.ranges, dtype=np.float64)
        key = (scan.header.frame_id, r.size, round(scan.angle_min, 6), round(scan.angle_increment, 8))
        if key not in self.ray_cache:
            a = scan.angle_min + scan.angle_increment * np.arange(r.size)
            dirs = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)])      # sensor frame
            self.ray_cache[key] = R @ dirs                                  # output frame
        ok = np.isfinite(r) & (r >= scan.range_min) & (r < scan.range_max - 1e-3)
        p = self.ray_cache[key][:, ok] * r[ok] + t[:, None]
        return p[0], p[1]

    def on_lidar(self, lidar):
        clouds = []
        pts = self.points(lidar)
        if pts is None:
            return
        clouds.append(pts)
        if self.use_us:
            now = Time.from_msg(lidar.header.stamp)
            for msg in self.latest_us.values():
                if now - Time.from_msg(msg.header.stamp) > self.max_age:
                    continue
                pts = self.points(msg)
                if pts is not None:
                    clouds.append(pts)

        x = np.concatenate([c[0] for c in clouds])
        y = np.concatenate([c[1] for c in clouds])
        d = np.hypot(x, y)
        keep = (d >= self.rmin) & (d <= self.rmax)
        x, y, d = x[keep], y[keep], d[keep]
        bins = np.floor((np.arctan2(y, x) + math.pi) / self.inc).astype(np.int64) % self.n_bins
        ranges = np.full(self.n_bins, np.inf)
        np.minimum.at(ranges, bins, d)          # nearest return per bin (overlap -> closest wins)

        out = LaserScan()
        out.header.stamp = lidar.header.stamp
        out.header.frame_id = self.frame
        out.angle_min = -math.pi
        out.angle_increment = self.inc
        out.angle_max = -math.pi + self.inc * (self.n_bins - 1)
        out.time_increment = 0.0
        out.scan_time = lidar.scan_time
        out.range_min = self.rmin
        out.range_max = self.rmax
        out.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(out)
        self.count += 1
        if self.count == 1:
            self.get_logger().info('first merged scan published')


def main():
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
