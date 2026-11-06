#!/usr/bin/env python3
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LaneCoverage(Node):
    def __init__(self):
        super().__init__('lane_coverage')
        for name, val in [
                ('coverage_width', 0.66), ('min_overlap', 0.06), ('wall_clearance', 0.15),
                ('end_clearance', 0.20), ('turn_margin', 0.05),
                ('footprint_front', 0.36), ('footprint_rear', 0.11), ('footprint_half_width', 0.33),
                ('lane_speed', 0.30), ('min_speed', 0.06), ('shift_speed', 0.15),
                ('max_turn_rate', 0.6), ('offset_gain', 1.5), ('heading_gain', 2.0),
                ('max_correction_deg', 20.0), ('lidar_fusion_gain', 0.15),
                ('obstacle_wait', 3.0), ('start_delay', 3.0), ('grid_resolution', 0.05),
                ('return_home', True), ('max_length', 30.0),
                ('report_dir', '~/mobile_robot/coverage_reports')]:
            self.declare_parameter(name, val)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cw, self.min_ov = p('coverage_width'), p('min_overlap')
        self.wall_clr, self.end_clr, self.turn_margin = p('wall_clearance'), p('end_clearance'), p('turn_margin')
        self.F, self.R, self.H = p('footprint_front'), p('footprint_rear'), p('footprint_half_width')
        self.corners = [(self.F, self.H), (self.F, -self.H), (-self.R, -self.H), (-self.R, self.H)]
        self.v_lane, self.v_min, self.v_shift = p('lane_speed'), p('min_speed'), p('shift_speed')
        self.w_max, self.k_off, self.k_head = p('max_turn_rate'), p('offset_gain'), p('heading_gain')
        self.max_corr = math.radians(p('max_correction_deg'))
        self.k_fuse, self.obs_wait, self.start_delay = p('lidar_fusion_gain'), p('obstacle_wait'), p('start_delay')
        self.res, self.return_home, self.max_len = p('grid_resolution'), p('return_home'), p('max_length')
        self.report_dir = os.path.expanduser(p('report_dir'))

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/coverage_status', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/coverage_grid', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage_markers', 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_merged', self.on_scan, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.odom = None
        self.pts = None
        self.state = 'WAIT'
        self.t_start = None
        self.queue = []
        self.prim = None
        self.bias_y = 0.0
        self.bias_psi = 0.0
        self.counting = False
        self.trail = []
        self.events = []
        self.min_clear = float('inf')
        self.dist = 0.0
        self.lidar_fixes = 0
        self.lidar_corr = []
        self.home_goal = None
        self.t_cover0 = None
        self.t_cover1 = None
        self.create_timer(0.05, self.step)
        self.create_timer(0.5, self.publish_viz)
        self.get_logger().info('lane coverage node ready - waiting for /odom and /scan_merged')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.odom is not None and self.counting:
            self.dist += math.hypot(x - self.odom[0], y - self.odom[1])
        self.odom = (x, y, yaw_of(q))

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + msg.angle_increment * np.arange(r.size)
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        self.pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
        if self.counting and self.pts.size:
            x, y = self.pts[:, 0], self.pts[:, 1]
            dx = np.maximum(np.maximum(-self.R - x, x - self.F), 0.0)
            dy = np.maximum(np.abs(y) - self.H, 0.0)
            self.min_clear = min(self.min_clear, float(np.min(np.hypot(dx, dy))))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def say(self, text, level='info'):
        if level == 'warn':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def cmd(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def pose_c(self):
        x, y, yaw = self.odom
        d = np.array([x, y]) - self.O
        return float(d @ self.u), float(d @ self.v) + self.bias_y, wrap(yaw - self.psi_c + self.bias_psi)

    def footprint_clearance(self, a, y, psi):
        c = float('inf')
        for cx, cy in self.corners:
            pa = a + cx * math.cos(psi) - cy * math.sin(psi)
            py = y + cx * math.sin(psi) + cy * math.cos(psi)
            c = min(c, py, self.W - py, pa - self.a_back, self.a_front - pa)
        return c

    def turn_clearance(self, a, y, psi, delta):
        return min(self.footprint_clearance(a, y, psi + delta * k) for k in np.linspace(0, 1, 37))

    def best_turn(self, a, y, psi, target):
        d = wrap(target - psi)
        options = [d] if abs(d) < 1e-3 else [d, d - math.copysign(2 * math.pi, d)]
        scored = [(self.turn_clearance(a, y, psi, o), -abs(o), o) for o in options]
        ok = [s for s in scored if s[0] >= self.turn_margin]
        best = max(ok, key=lambda s: s[1]) if ok else max(scored)
        return best[2], best[0]

    def measure(self):
        P = self.pts[np.hypot(self.pts[:, 0], self.pts[:, 1]) < 12.0]
        bins = np.arange(-8.0, 8.0, 0.05)
        centers = (bins[:-1] + bins[1:]) / 2

        def pair_score(th):
            hist, _ = np.histogram(P @ np.array([-math.sin(th), math.cos(th)]), bins=bins)
            return hist[centers < -0.2].max(initial=0) + hist[centers > 0.2].max(initial=0)

        coarse = np.radians(np.arange(-90.0, 90.0, 0.5))
        th = max(coarse, key=lambda t: (pair_score(t), -abs(t)))
        fine = th + np.radians(np.arange(-0.6, 0.61, 0.05))
        th = float(max(fine, key=lambda t: (pair_score(t), -abs(t - th))))
        u = np.array([math.cos(th), math.sin(th)])
        n = np.array([-math.sin(th), math.cos(th)])
        off = P @ n
        hist, _ = np.histogram(off, bins=bins)
        neg, pos = centers < -0.2, centers > 0.2
        if hist[neg].max(initial=0) < 15 or hist[pos].max(initial=0) < 15:
            return None
        d_r = -centers[neg][np.argmax(hist[neg])]
        d_l = centers[pos][np.argmax(hist[pos])]
        d_r = -float(np.median(off[np.abs(off + d_r) < 0.1]))
        d_l = float(np.median(off[np.abs(off - d_l) < 0.1]))
        along = P @ u
        band = (off > -d_r + 0.15) & (off < d_l - 0.15)

        def end(sign):
            vals = sign * along[band & (sign * along > 0.3)]
            if vals.size < 5:
                return None
            h, e = np.histogram(vals, bins=np.arange(0.3, 13.0, 0.05))
            idx = np.nonzero(h >= 5)[0]
            if idx.size == 0:
                return None
            k = idx[-1]
            sel = (vals >= e[k] - 0.1) & (vals <= e[k + 1] + 0.1)
            return float(np.median(vals[sel]))

        return th, d_r, d_l, end(+1), end(-1)

    def lidar_fix(self, a, y, psi):
        if self.pts is None or len(self.pts) < 30:
            return None
        P = self.pts[np.hypot(self.pts[:, 0], self.pts[:, 1]) < 4.0]
        best = None
        for dth in np.radians(np.arange(-6.0, 6.01, 0.5)):
            th = -psi + dth
            u = np.array([math.cos(th), math.sin(th)])
            n = np.array([-math.sin(th), math.cos(th)])
            along, off = P @ u, P @ n
            local = np.abs(along) < 2.5
            r = local & (np.abs(off + y) < 0.25)
            l = local & (np.abs(off - (self.W - y)) < 0.25)
            parts = [s for s in (off[r], off[l]) if s.size >= 15]
            if not parts:
                continue
            spread = float(np.mean([np.std(s) for s in parts]))
            if best is None or spread < best[0]:
                best = (spread, th, off[r], off[l])
        if best is None or best[0] > 0.05:
            return None
        _, th, offr, offl = best
        ys = []
        if offr.size >= 15:
            ys.append(-float(np.median(offr)))
        if offl.size >= 15:
            ys.append(self.W - float(np.median(offl)))
        return float(np.mean(ys)), -th

    def gap_ahead(self):
        if self.pts is None:
            return float('inf'), None
        x, y = self.pts[:, 0], self.pts[:, 1]
        sel = np.nonzero((x > self.F - 0.05) & (np.abs(y) < self.H + 0.03))[0]
        if sel.size == 0:
            return float('inf'), None
        k = sel[np.argmin(x[sel])]
        return float(x[k] - self.F), (round(float(x[k]), 2), round(float(y[k]), 2))

    def plan(self, m):
        th, d_r, d_l, f, b = m
        x, y, yaw = self.odom
        self.W = d_r + d_l
        self.psi_c = wrap(yaw + th)
        self.u = np.array([math.cos(self.psi_c), math.sin(self.psi_c)])
        self.v = np.array([-math.sin(self.psi_c), math.cos(self.psi_c)])
        self.O = np.array([x, y]) - d_r * self.v
        self.a_front = f if f is not None else self.max_len
        self.a_back = -b if b is not None else -0.5
        self.L = self.a_front - self.a_back

        first = self.wall_clr + self.cw / 2.0
        last = self.W - first
        if last <= first:
            self.lanes = [self.W / 2.0]
        else:
            n = int(math.ceil((last - first) / (self.cw - self.min_ov) - 1e-6)) + 1
            self.lanes = list(np.linspace(first, last, n))
        self.spacing = (self.lanes[1] - self.lanes[0]) if len(self.lanes) > 1 else 0.0
        ov = self.cw - self.spacing if len(self.lanes) > 1 else 0.0

        self.say(f'corridor measured: width {self.W:.2f} m (right wall {d_r:.2f} m, left wall '
                 f'{d_l:.2f} m), length {self.L:.2f} m '
                 f'(front end {"%.2f" % f if f is not None else "not seen"}, back end '
                 f'{"%.2f" % b if b is not None else "not seen"}), direction {math.degrees(th):+.1f} deg')
        self.say(f'plan: {len(self.lanes)} lanes, spacing {self.spacing:.3f} m, overlap {ov:.3f} m '
                 f'({100 * ov / self.cw:.0f} % of coverage width), offsets from right wall: '
                 + ', '.join(f'{o:.2f}' for o in self.lanes))

        self.na = int(math.ceil(self.L / self.res))
        self.ny = int(math.ceil(self.W / self.res))
        ga = self.a_back + (np.arange(self.na) + 0.5) * self.res
        gy = (np.arange(self.ny) + 0.5) * self.res
        self.GA, self.GY = np.meshgrid(ga, gy)
        self.count = np.zeros(self.GA.shape, dtype=np.int32)
        self.last_seen = np.full(self.GA.shape, -1e9)

        o0 = self.lanes[0]
        q = [('rotate', -math.pi / 2), ('shift', o0, -math.pi / 2, math.pi), ('rotate', math.pi),
             ('lane', -1, -1, o0), ('rotate', 0.0), ('start_count',)]
        for i, o in enumerate(self.lanes):
            d = 1 if i % 2 == 0 else -1
            q.append(('lane', i, d, o))
            if i + 1 < len(self.lanes):
                nxt = 0.0 if d < 0 else math.pi
                q += [('rotate', math.pi / 2), ('shift', self.lanes[i + 1], math.pi / 2, nxt),
                      ('rotate', nxt)]
        q += [('stop_count',), ('home',), ('report',)]
        self.queue = q

    def step(self):
        if self.state == 'WAIT':
            if self.odom is None or self.pts is None:
                return
            if self.t_start is None:
                self.t_start = self.now()
            if self.now() - self.t_start < self.start_delay:
                return
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_ground', Time())
                t, r = tf.transform.translation, tf.transform.rotation
                self.home_goal = ('map', t.x, t.y, yaw_of(r))
            except TransformException:
                x, y, yaw = self.odom
                self.home_goal = ('odom', x, y, yaw)
            self.say(f'Home = ({self.home_goal[1]:.2f}, {self.home_goal[2]:.2f}) in {self.home_goal[0]}')
            self.state = 'MEASURE'
            return
        if self.state == 'MEASURE':
            m = self.measure()
            if m is None:
                self.say('cannot see two side walls yet - retrying', 'warn')
                return
            self.plan(m)
            self.state = 'RUN'
            return
        if self.state != 'RUN':
            return
        if self.counting:
            self.mark_coverage()
        if self.prim is None:
            if not self.queue:
                self.state = 'DONE'
                self.cmd()
                return
            self.prim = {'p': self.queue.pop(0), 'init': False}
        if getattr(self, 'do_' + self.prim['p'][0])(self.prim):
            self.cmd()
            self.prim = None

    def do_start_count(self, pr):
        self.counting = True
        self.t_cover0 = self.now()
        self.say('coverage started')
        return True

    def do_stop_count(self, pr):
        self.counting = False
        self.t_cover1 = self.now()
        return True

    def do_rotate(self, pr):
        a, y, psi = self.pose_c()
        if not pr['init']:
            delta, clr = self.best_turn(a, y, psi, pr['p'][1])
            if clr < self.turn_margin:
                self.say(f'turn clearance only {clr:.2f} m', 'warn')
            pr.update(init=True, delta=delta, done=0.0, prev=self.odom[2])
        yaw = self.odom[2]
        pr['done'] += wrap(yaw - pr['prev'])
        pr['prev'] = yaw
        rem = pr['delta'] - pr['done']
        if abs(rem) < math.radians(1.0):
            return True
        self.cmd(0.0, math.copysign(min(self.w_max, max(0.15, 1.5 * abs(rem))), rem))
        return False

    def do_shift(self, pr):
        _, target, heading, next_heading = pr['p']
        a, y, psi = self.pose_c()
        if not pr['init']:
            s = math.copysign(1.0, math.sin(heading))
            t = target
            for _ in range(60):
                if self.best_turn(a, t, heading, next_heading)[1] >= self.turn_margin:
                    break
                t -= s * 0.01
            if abs(t - target) > 0.005:
                self.get_logger().info(f'lane change stops at y={t:.2f} instead of {target:.2f} '
                                       f'(turn clearance), lane keeping closes the rest')
            pr.update(init=True, target=t, s=s)
        fix = self.lidar_fix(a, y, psi)
        if fix:
            self.fuse(fix, y, psi)
            a, y, psi = self.pose_c()
        rem = (pr['target'] - y) * pr['s']
        if rem < 0.01:
            return True
        if self.gap_ahead()[0] < 0.10:
            self.cmd()
            return False
        v = min(self.v_shift, max(0.04, 0.8 * rem))
        self.cmd(v, max(-self.w_max, min(self.w_max, self.k_head * wrap(heading - psi))))
        return False

    def do_lane(self, pr):
        _, idx, d, offset = pr['p']
        a, y, psi = self.pose_c()
        if not pr['init']:
            pr.update(init=True, blocked_since=None, a0=a)
            if idx >= 0:
                self.say(f'lane {idx + 1}/{len(self.lanes)} '
                         f'({"forward" if d > 0 else "back"}, {offset:.2f} m from right wall)')
            else:
                self.say('moving to the start corner')
        fix = self.lidar_fix(a, y, psi)
        if fix:
            self.fuse(fix, y, psi)
            a, y, psi = self.pose_c()
        psi_d = 0.0 if d > 0 else math.pi
        gap, hit = self.gap_ahead()
        geo = (self.a_front - (a + self.F)) if d > 0 else ((a - self.F) - self.a_back)
        if gap <= self.end_clr:
            if geo < self.end_clr + 0.4:
                self.events.append(f'lane {idx + 1}: {abs(a - pr["a0"]):.2f} m' if idx >= 0 else
                                   'start corner reached')
                return True
            if pr['blocked_since'] is None:
                pr['blocked_since'] = self.now()
                self.say(f'obstacle {gap:.2f} m ahead in lane {idx + 1} (point {hit} in robot frame, '
                         f'{geo:.2f} m before the end wall) - waiting', 'warn')
            self.cmd()
            if self.now() - pr['blocked_since'] > self.obs_wait:
                self.events.append(f'lane {idx + 1}: ended early at a={a:.2f} m (obstacle), '
                                   f'{geo:.2f} m of lane not covered')
                self.say(f'lane {idx + 1} blocked - ending it early', 'warn')
                return True
            return False
        pr['blocked_since'] = None
        corr = max(-self.max_corr, min(self.max_corr, math.atan(self.k_off * (y - offset))))
        err = wrap(psi_d - d * corr - psi)
        w = max(-self.w_max, min(self.w_max, self.k_head * err))
        v = min(self.v_lane, max(self.v_min, 0.8 * (gap - self.end_clr)))
        self.cmd(v * max(0.3, math.cos(err)), w)
        if self.counting:
            self.trail.append(tuple(self.odom[:2]))
        return False

    def do_home(self, pr):
        if not self.return_home:
            return True
        if not pr['init']:
            pr['init'] = True
            pr['t0'] = time.time()
            self.say('all lanes done - returning Home with Nav2')
            if not self.nav.wait_for_server(timeout_sec=10.0):
                self.say('Nav2 not available - staying here', 'warn')
                self.events.append('return Home: Nav2 not available')
                return True
            frame, x, y, yaw = self.home_goal
            g = NavigateToPose.Goal()
            g.pose = PoseStamped()
            g.pose.header.frame_id = frame
            g.pose.header.stamp = self.get_clock().now().to_msg()
            g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
            g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            pr['future'] = self.nav.send_goal_async(g)
            pr['stage'] = 'sent'
            return False
        if pr['stage'] == 'sent' and pr['future'].done():
            gh = pr['future'].result()
            if not gh.accepted:
                self.events.append('return Home: goal rejected')
                return True
            pr['res'] = gh.get_result_async()
            pr['stage'] = 'driving'
        if pr['stage'] == 'driving' and pr['res'].done():
            ok = pr['res'].result().status == 4
            frame, hx, hy, _ = self.home_goal
            try:
                tf = self.tf_buffer.lookup_transform(frame, 'base_ground', Time())
                err = math.hypot(tf.transform.translation.x - hx, tf.transform.translation.y - hy)
            except TransformException:
                err = float('nan')
            self.events.append(f'return Home: {"reached" if ok else "FAILED"}, {err:.2f} m from Home')
            return True
        return time.time() - pr['t0'] > 600.0

    def do_report(self, pr):
        inner = ((self.GY >= self.wall_clr) & (self.GY <= self.W - self.wall_clr) &
                 (self.GA >= self.a_back + self.end_clr) & (self.GA <= self.a_front - self.end_clr))
        cov = self.count >= 1
        over = self.count >= 2
        t = (self.t_cover1 or self.now()) - (self.t_cover0 or self.now())
        lines = [
            '=========== CORRIDOR COVERAGE REPORT ===========',
            f'corridor          : {self.W:.2f} m wide x {self.L:.2f} m long ({self.W * self.L:.1f} m2)',
            f'lanes             : {len(self.lanes)} (spacing {self.spacing:.3f} m, coverage width {self.cw:.2f} m)',
            f'coverage          : {100 * cov.mean():.1f} % of the whole floor',
            f'                    {100 * cov[inner].mean():.1f} % of the floor inside the wall clearance band '
            f'({self.wall_clr:.2f} m sides / {self.end_clr:.2f} m ends)',
            f'overlap           : {100 * over[cov].mean():.1f} % of the covered floor was covered twice or more '
            f'({over.sum() * self.res ** 2:.1f} m2)',
            f'min wall clearance: {self.min_clear:.3f} m (closest any wall got to the robot footprint, lidar)',
            f'distance / time   : {self.dist:.1f} m in {t:.0f} s (sim time)',
            f'lidar corrections : {self.lidar_fixes} wall fixes, mean |offset correction| '
            f'{1000 * np.mean(self.lidar_corr) if self.lidar_corr else 0:.1f} mm',
        ] + ['  - ' + e for e in self.events] + ['================================================']
        for ln in lines:
            self.get_logger().info(ln)
        os.makedirs(self.report_dir, exist_ok=True)
        path = os.path.join(self.report_dir, time.strftime('coverage_%Y%m%d_%H%M%S.txt'))
        with open(path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        self.say(f'DONE - coverage {100 * cov.mean():.1f} %, overlap {100 * over[cov].mean():.1f} %, '
                 f'report {path}')
        return True

    def fuse(self, fix, y_now, psi_now):
        y_l, psi_l = fix
        dy, dpsi = y_l - y_now, wrap(psi_l - psi_now)
        if abs(dy) > 0.3 or abs(dpsi) > math.radians(10):
            return
        self.bias_y += self.k_fuse * dy
        self.bias_psi += self.k_fuse * dpsi
        self.lidar_fixes += 1
        self.lidar_corr.append(abs(dy))

    def mark_coverage(self):
        a, y, psi = self.pose_c()
        c, s = math.cos(psi), math.sin(psi)
        win = (np.abs(self.GA - a) < 0.6) & (np.abs(self.GY - y) < 0.6)
        dA, dY = self.GA[win] - a, self.GY[win] - y
        xb = dA * c + dY * s
        yb = -dA * s + dY * c
        inside = (xb >= -self.R) & (xb <= self.F) & (np.abs(yb) <= self.cw / 2.0)
        idx = np.nonzero(win)
        rows, cols = idx[0][inside], idx[1][inside]
        now = self.now()
        fresh = now - self.last_seen[rows, cols] > 2.0
        self.count[rows[fresh], cols[fresh]] += 1
        self.last_seen[rows, cols] = now

    def publish_viz(self):
        if self.state not in ('RUN', 'DONE'):
            return
        stamp = self.get_clock().now().to_msg()
        g = OccupancyGrid()
        g.header.frame_id, g.header.stamp = 'odom', stamp
        g.info.resolution = self.res
        g.info.width, g.info.height = self.na, self.ny
        o = self.O + self.a_back * self.u
        g.info.origin.position.x, g.info.origin.position.y = float(o[0]), float(o[1])
        g.info.origin.orientation.z = math.sin(self.psi_c / 2)
        g.info.origin.orientation.w = math.cos(self.psi_c / 2)
        data = np.zeros(self.count.shape, dtype=np.int8)
        data[self.count == 1] = 15
        data[self.count >= 2] = 90
        g.data = data.flatten().tolist()
        self.grid_pub.publish(g)

        ma = MarkerArray()
        lanes = Marker()
        lanes.header.frame_id, lanes.header.stamp = 'odom', stamp
        lanes.ns, lanes.id, lanes.type = 'lanes', 0, Marker.LINE_LIST
        lanes.scale.x = 0.02
        lanes.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.8)
        lanes.pose.orientation.w = 1.0
        for off in self.lanes:
            for aa in (self.a_back + self.F + self.end_clr, self.a_front - self.F - self.end_clr):
                p = self.O + aa * self.u + off * self.v
                lanes.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.05))
        ma.markers.append(lanes)
        trail = Marker()
        trail.header.frame_id, trail.header.stamp = 'odom', stamp
        trail.ns, trail.id, trail.type = 'trail', 1, Marker.LINE_STRIP
        trail.scale.x = 0.03
        trail.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        trail.pose.orientation.w = 1.0
        trail.points = [Point(x=float(x), y=float(y), z=0.06) for x, y in self.trail[::5]]
        if len(trail.points) >= 2:
            ma.markers.append(trail)
        txt = Marker()
        txt.header.frame_id, txt.header.stamp = 'odom', stamp
        txt.ns, txt.id, txt.type = 'state', 2, Marker.TEXT_VIEW_FACING
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = self.odom[0], self.odom[1], 1.0
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.3
        txt.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
        name = self.prim['p'][0] if self.prim else self.state
        txt.text = f'{name}  coverage {100 * (self.count >= 1).mean():.0f} %'
        ma.markers.append(txt)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = LaneCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
