#!/usr/bin/env python3
"""Draw the true outline of the world's walls in RViz (display only).

Reads a ground-truth map made from the Isaac Sim world itself (scripts/isaac_omap.py ->
maps/<world>_isaac.yaml) and publishes the edges of its walls as a Marker on /map_outline,
in the map frame. Nothing else uses it: SLAM, AMCL and Nav2 never see this map.

Use: compare it with the live SLAM map / the robot in RViz. If the SLAM walls lie on the
outline, the robot is localized correctly; if the robot is drawn outside the outline, the
position estimate has drifted.

Works because the robot spawns at the Isaac world origin facing +X, so map = world frame.
"""
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    fields, pos = [], 0
    while len(fields) < 4:                       # magic, width, height, maxval (skip comments)
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    w, h = int(fields[1]), int(fields[2])
    pos += 1
    return np.frombuffer(data[pos:pos + w * h], dtype=np.uint8).reshape(h, w)


class MapOutline(Node):
    def __init__(self):
        super().__init__('map_outline')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('color', [1.0, 0.6, 0.0, 0.9])   # orange
        self.declare_parameter('z', 0.03)

        path = os.path.expanduser(self.get_parameter('map_yaml').value)
        if not path or not os.path.isfile(path):
            self.get_logger().error(f'map_yaml not found: "{path}" - outline disabled')
            self.marker = None
            return
        with open(path) as f:
            info = yaml.safe_load(f)
        img = read_pgm(os.path.join(os.path.dirname(path), info['image']))
        if info.get('negate', 0):
            img = 255 - img
        res = float(info['resolution'])
        ox, oy = float(info['origin'][0]), float(info['origin'][1])
        occ = (255 - img) / 255.0 >= float(info.get('occupied_thresh', 0.65))

        # keep only wall cells next to a non-wall cell -> thin outline of every wall
        pad = np.pad(occ, 1, constant_values=False)
        inner = pad[:-2, 1:-1] & pad[2:, 1:-1] & pad[1:-1, :-2] & pad[1:-1, 2:]
        edge = occ & ~inner
        rows, cols = np.nonzero(edge)
        h = img.shape[0]
        xs = ox + (cols + 0.5) * res
        ys = oy + (h - 1 - rows + 0.5) * res                 # image row 0 is the top (max y)

        z = float(self.get_parameter('z').value)
        r, g, b, a = self.get_parameter('color').value
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').value
        m.ns = 'map_outline'
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = res
        m.scale.z = 0.02
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        m.points = [Point(x=float(x), y=float(y), z=z) for x, y in zip(xs, ys)]
        m.frame_locked = True
        self.marker = m

        self.pub = self.create_publisher(Marker, '/map_outline', 1)
        self.create_timer(1.0, self.publish)    # re-sent every second, so RViz always has it
        self.get_logger().info(f'outline of {os.path.basename(path)}: {len(m.points)} wall-edge '
                               f'cells -> /map_outline (frame {m.header.frame_id}, display only)')

    def publish(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)


def main():
    rclpy.init()
    node = MapOutline()
    try:
        if node.marker is not None:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
