"""Generate a simple maze world (.usd) for the amr robot.

  python3 make_maze.py                         # 5 x 5 cells of 2 m -> urdf/world_maze.usd
  python3 make_maze.py --rows 4 --cols 6 --cell 2.2 --seed 7 --out ~/mobile_robot/src/amr/urdf/maze2.usd

(any Python with the USD "pxr" module works, also ~/isaacsim_5.1/python.sh)

Layout: a "perfect" maze (exactly one route between any two cells, with branches and dead
ends) on a grid of square cells. The robot's start cell is the bottom-left one, and its centre
is the world origin (0, 0), so the robot (spawned at the origin facing +X) starts inside the
maze = Home. Every wall is a cube with a PhysX collider (the lidar/ultrasonics see them).

Sizes for this robot: 0.66 m wide, lidar at 0.28 m, ultrasonics at 0.12 m
  cell 2.0 m  -> corridor 1.85 m free (Nav2 keeps ~0.5 m from walls), walls 0.8 m high.
Open the file in Isaac Sim to move/add walls or boxes; keep colliders on everything you add.
"""
import argparse
import os
import random

import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_utils import WALL_BLACK, material, new_world, solid_box  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--rows", type=int, default=5, help="cells along +Y")
ap.add_argument("--cols", type=int, default=5, help="cells along +X")
ap.add_argument("--cell", type=float, default=2.0, help="cell size [m] (corridor = cell - wall)")
ap.add_argument("--wall", type=float, default=0.15, help="wall thickness [m]")
ap.add_argument("--height", type=float, default=0.8, help="wall height [m]")
ap.add_argument("--seed", type=int, default=3, help="change it for a different maze")
ap.add_argument("--loops", type=int, default=1, help="extra openings (0 = perfect maze)")
ap.add_argument("--out", default=os.path.expanduser("~/mobile_robot/src/amr/urdf/world_maze.usd"))
args = ap.parse_args()

R, C, S = args.rows, args.cols, args.cell
rng = random.Random(args.seed)

# walls between cells: h[r][c] = wall on the south side of cell (r, c) (r = 0..R),
#                      v[r][c] = wall on the west side of cell (r, c)  (c = 0..C)
h = [[True] * C for _ in range(R + 1)]
v = [[True] * (C + 1) for _ in range(R)]

# depth-first "recursive backtracker" from the start cell -> perfect maze
seen = {(0, 0)}
stack = [(0, 0)]
while stack:
    r, c = stack[-1]
    nbrs = [(r + dr, c + dc, dr, dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 0 <= r + dr < R and 0 <= c + dc < C and (r + dr, c + dc) not in seen]
    if not nbrs:
        stack.pop()
        continue
    nr, nc, dr, dc = rng.choice(nbrs)
    if dr == 1:
        h[r + 1][c] = False
    elif dr == -1:
        h[r][c] = False
    elif dc == 1:
        v[r][c + 1] = False
    else:
        v[r][c] = False
    seen.add((nr, nc))
    stack.append((nr, nc))

# a few extra openings give loops (good for SLAM loop closure)
inner = [("h", r, c) for r in range(1, R) for c in range(C) if h[r][c]] + \
        [("v", r, c) for r in range(R) for c in range(1, C) if v[r][c]]
for kind, r, c in rng.sample(inner, min(args.loops, len(inner))):
    (h if kind == "h" else v)[r][c] = False

# start cell (0, 0) centred on the world origin
x0, y0 = -S / 2.0, -S / 2.0

stage = new_world(args.out, "/World/Maze/Sun")
black = material(stage, "WallBlack", WALL_BLACK)

n = 0


def wall(cx, cy, sx, sy, _kind):
    global n
    solid_box(stage, f"/World/Maze/wall_{n:03d}", (cx, cy, args.height / 2.0),
              (sx, sy, args.height), black, WALL_BLACK)
    n += 1


OUTER = INNER = None          # all walls black (material), kept as names for readability
t = args.wall
for r in range(R + 1):                      # horizontal walls (along X), merged into runs
    c = 0
    while c < C:
        if not h[r][c]:
            c += 1
            continue
        c1 = c
        while c1 < C and h[r][c1]:
            c1 += 1
        wall(x0 + (c + c1) * S / 2.0, y0 + r * S, (c1 - c) * S + t, t,
             OUTER if r in (0, R) else INNER)
        c = c1
for c in range(C + 1):                      # vertical walls (along Y), merged into runs
    r = 0
    while r < R:
        if not v[r][c]:
            r += 1
            continue
        r1 = r
        while r1 < R and v[r1][c]:
            r1 += 1
        wall(x0 + c * S, y0 + (r + r1) * S / 2.0, t, (r1 - r) * S + t,
             OUTER if c in (0, C) else INNER)
        r = r1

stage.GetRootLayer().Save()

# top view in the terminal (H = Home / robot start)
print(f"maze {R} x {C} cells of {S} m, corridor {S - t:.2f} m, walls {args.height} m high, "
      f"{n} wall prims -> {args.out}")
print(f"area x {x0:.1f} .. {x0 + C * S:.1f} m, y {y0:.1f} .. {y0 + R * S:.1f} m")
for r in range(R, -1, -1):
    print("+" + "+".join("---" if h[r][c] else "   " for c in range(C)) + "+")
    if r == 0:
        break
    rr = r - 1
    row = ""
    for c in range(C + 1):
        row += "|" if v[rr][c] else " "
        if c < C:
            row += " H " if (rr, c) == (0, 0) else "   "
    print(row)
