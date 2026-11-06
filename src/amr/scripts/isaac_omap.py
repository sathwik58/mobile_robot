"""Make a Nav2 occupancy map (.pgm + .yaml) straight from an Isaac Sim world.

The map is built from the PhysX colliders (the same thing the PhysX lidar sees), cut as a
horizontal slab between --zmin and --zmax. The map frame equals the Isaac world frame, so a
robot that spawns at the world origin starts at map (0, 0, 0).

  ~/isaacsim_5.1/python.sh isaac_omap.py --world ~/mobile_robot/src/amr/urdf/world1.usd \
      --out ~/mobile_robot/src/amr_nav/maps/world1
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--world", required=True, help="world .usd (without the robot)")
parser.add_argument("--out", required=True, help="output path without extension")
parser.add_argument("--res", type=float, default=0.05, help="cell size [m]")
parser.add_argument("--zmin", type=float, default=0.05, help="lowest obstacle height kept [m]")
parser.add_argument("--zmax", type=float, default=0.60, help="highest obstacle height kept [m]")
parser.add_argument("--bounds", type=float, nargs=4, metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                    help="map area in world metres (default: whole world, +1 m border)")
parser.add_argument("--skip", nargs="*", default=["/World/amr", "/World/GroundPlane"],
                    help="prims to leave out (robot, ground)")
args, _ = parser.parse_known_args()

world = os.path.abspath(os.path.expanduser(args.world))
out = os.path.abspath(os.path.expanduser(args.out))
if not os.path.isfile(world):
    sys.exit(f"[omap] world not found: {world}")
os.makedirs(os.path.dirname(out), exist_ok=True)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.asset.gen.omap")
app.update()

import numpy as np  # noqa: E402
import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.asset.gen.omap.bindings import _omap  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from isaac_common import localize_cloud_assets  # noqa: E402

print(f"[omap] opening {world}", flush=True)
ctx = omni.usd.get_context()
ctx.open_stage(world)
while is_stage_loading():
    app.update()
stage = ctx.get_stage()
if stage is None:
    app.close()
    sys.exit(f"[omap] could not open {world}")
n = localize_cloud_assets(stage)
if n:
    print(f"[omap] {n} cloud asset paths -> local copy", flush=True)
    for _ in range(5):
        app.update()
    while is_stage_loading():
        app.update()

for path in args.skip:
    prim = stage.GetPrimAtPath(path)
    if prim:
        prim.SetActive(False)
        print(f"[omap] left out {path}", flush=True)
for p in stage.Traverse():  # ground planes would fill the whole slab
    if p.GetName().lower() in ("groundplane", "ground_plane", "defaultgroundplane"):
        p.SetActive(False)

if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
    UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

if args.bounds:
    xmin, ymin, xmax, ymax = args.bounds
else:
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    xmin, ymin, xmax, ymax = lo[0] - 1.0, lo[1] - 1.0, hi[0] + 1.0, hi[1] + 1.0
# snap to the grid so cell centres are stable
r = args.res
xmin, ymin = np.floor(xmin / r) * r, np.floor(ymin / r) * r
xmax, ymax = np.ceil(xmax / r) * r, np.ceil(ymax / r) * r
print(f"[omap] area x {xmin:.2f}..{xmax:.2f}  y {ymin:.2f}..{ymax:.2f}  "
      f"z {args.zmin:.2f}..{args.zmax:.2f}  res {r}", flush=True)

OCC, FREE, UNK = 1.0, 0.0, 0.5
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(10):
    app.update()
gen = _omap.Generator(omni.physx.get_physx_interface(), ctx.get_stage_id())
gen.update_settings(r, OCC, FREE, UNK)
gen.set_transform((0.0, 0.0, 0.0), (xmin, ymin, args.zmin), (xmax, ymax, args.zmax))
gen.generate2d()
app.update()
timeline.stop()

dims = gen.get_dimensions()
buf = np.array(gen.get_buffer(), dtype=np.float32)
if buf.size != dims[0] * dims[1]:
    app.close()
    sys.exit(f"[omap] empty map (buffer {buf.size}, dims {dims}) - no colliders in the area?")
# buffer rows run along +y from the min bound, columns along -x from the max bound
# (checked with test cubes), so flip the columns to get x increasing to the right
grid = np.fliplr(buf.reshape(dims[1], dims[0]))
mn, mx = gen.get_min_bound(), gen.get_max_bound()
ox, oy = float(mx[0]) - dims[0] * r, float(mn[1])

# Nav2 / map_server trinary PGM: 0 = occupied, 254 = free, 205 = unknown. Row 0 is the top (max y).
img = np.full(grid.shape, 205, dtype=np.uint8)
img[grid == FREE] = 254
img[grid == OCC] = 0
img = np.flipud(img)

with open(out + ".pgm", "wb") as f:
    f.write(f"P5\n# isaac_omap {os.path.basename(world)}\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
    f.write(img.tobytes())
with open(out + ".yaml", "w") as f:
    f.write(f"image: {os.path.basename(out)}.pgm\n"
            f"mode: trinary\n"
            f"resolution: {r}\n"
            f"origin: [{ox:.4f}, {oy:.4f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n")
occ = int((grid == OCC).sum())
free = int((grid == FREE).sum())
print(f"[omap] {img.shape[1]}x{img.shape[0]} cells, occupied {occ}, free {free}, "
      f"unknown {grid.size - occ - free}", flush=True)
print(f"[omap] wrote {out}.pgm and {out}.yaml (origin {ox:.2f}, {oy:.2f})", flush=True)
app.close()
