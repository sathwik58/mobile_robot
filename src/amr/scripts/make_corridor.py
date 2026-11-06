"""Generate a straight corridor world (.usd) for the lane-coverage task (Part C).

  python3 make_corridor.py                               # 3.3 m x 12 m -> urdf/world_corridor.usd
  python3 make_corridor.py --width 2.8 --length 10 --box 5,0.6,0.5 --out ~/.../corridor2.usd

The robot spawns at the world origin facing +X, so it starts inside the corridor, on its
centre line, `--back` metres in front of the back wall. Inner wall surfaces are exactly at
y = +-width/2, x = -back and x = length - back. Every wall has a PhysX collider.
--box x,y,size adds a square box obstacle (repeat for more); none by default.
"""
import argparse
import os

import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_utils import BOX_ORANGE, WALL_BLACK, material, new_world, solid_box  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--width", type=float, default=3.3, help="free width between the walls [m]")
ap.add_argument("--length", type=float, default=12.0, help="free length between the end walls [m]")
ap.add_argument("--back", type=float, default=1.0, help="distance from robot start to back wall [m]")
ap.add_argument("--wall", type=float, default=0.15, help="wall thickness [m]")
ap.add_argument("--height", type=float, default=0.8, help="wall height [m]")
ap.add_argument("--box", action="append", default=[], help="obstacle box x,y,size (world m)")
ap.add_argument("--out", default=os.path.expanduser("~/mobile_robot/src/amr/urdf/world_corridor.usd"))
args = ap.parse_args()

W, L, B, T, Hh = args.width, args.length, args.back, args.wall, args.height
stage = new_world(args.out, "/World/Corridor/Sun")
black = material(stage, "WallBlack", WALL_BLACK)
orange = material(stage, "BoxOrange", BOX_ORANGE)


def box(name, cx, cy, sx, sy, sz, mat, rgb):
    solid_box(stage, f"/World/Corridor/{name}", (cx, cy, sz / 2.0), (sx, sy, sz), mat, rgb)


grey = (black, WALL_BLACK)
x_back, x_front = -B, L - B
x_mid = (x_back + x_front) / 2.0
box("wall_right", x_mid, -W / 2 - T / 2, L + 2 * T, T, Hh, *grey)
box("wall_left", x_mid, W / 2 + T / 2, L + 2 * T, T, Hh, *grey)
box("wall_back", x_back - T / 2, 0.0, T, W, Hh, *grey)
box("wall_front", x_front + T / 2, 0.0, T, W, Hh, *grey)
for i, spec in enumerate(args.box):
    bx, by, bs = (float(v) for v in spec.split(","))
    box(f"box_{i}", bx, by, bs, bs, 0.5, orange, BOX_ORANGE)
stage.GetRootLayer().Save()
print(f"corridor {W} m x {L} m (x {x_back:.2f} .. {x_front:.2f}, y {-W/2:.2f} .. {W/2:.2f}), "
      f"{len(args.box)} boxes -> {args.out}")
