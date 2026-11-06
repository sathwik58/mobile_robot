"""Start Isaac Sim 5.1, open the amr stage, add a ground plane and press Play.

Run with Isaac Sim's own Python (isaac.launch.py does this for you):
  ~/isaacsim_5.1/python.sh isaac_run.py --usd ~/mobile_robot/src/amr/urdf/amr.usd

The stage already holds the robot, its sensors and the ROS 2 action graph, so once
Play starts the robot listens on /cmd_vel and publishes /scan, /odom, /tf, ...
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--usd", required=True, help="stage to open (amr.usd)")
parser.add_argument("--world", default="", help="world .usd to load around the robot (optional)")
parser.add_argument("--headless", action="store_true", help="run without a window")
args, _ = parser.parse_known_args()

usd_path = os.path.abspath(os.path.expanduser(args.usd))
if not os.path.isfile(usd_path):
    sys.exit(f"[isaac_run] stage not found: {usd_path}")

from isaacsim import SimulationApp  # noqa: E402  (must come before other omni imports)

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.physx", "isaacsim.sensors.physics",
            "isaacsim.robot.wheeled_robots", "isaacsim.core.nodes"):
    enable_extension(ext)
app.update()

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402

print(f"[isaac_run] opening {usd_path}", flush=True)
omni.usd.get_context().open_stage(usd_path)
while is_stage_loading():
    app.update()
stage = omni.usd.get_context().get_stage()

# optional world (warehouse, rooms, ...) referenced under /World/Env; robot stays at the origin
if args.world:
    world_path = os.path.abspath(os.path.expanduser(args.world))
    if not os.path.isfile(world_path):
        sys.exit(f"[isaac_run] world not found: {world_path}")
    from pxr import Sdf, UsdGeom  # noqa: E402

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from isaac_common import localize_layer  # noqa: E402

    world_layer = Sdf.Layer.FindOrOpen(world_path)  # kept alive so the in-memory fix is used
    n = localize_layer(world_layer)
    env = UsdGeom.Xform.Define(stage, "/World/Env").GetPrim()
    env.GetReferences().AddReference(world_path, "/World")
    for _ in range(5):
        app.update()
    while is_stage_loading():
        app.update()
    print(f"[isaac_run] world {world_path} loaded under /World/Env ({n} cloud paths -> local)",
          flush=True)

# ground plane (the saved stage has none); only added in memory, the file is not changed
has_ground = any(p.GetName().lower() in ("groundplane", "ground_plane", "defaultgroundplane")
                 for p in stage.Traverse())
if not has_ground:
    from isaacsim.core.api.objects import GroundPlane
    GroundPlane(prim_path="/World/GroundPlane", z_position=0.0, size=50.0)
    print("[isaac_run] ground plane added at z = 0", flush=True)

# wheel joints must be velocity drives (stiffness 0) for /cmd_vel to turn them; the URDF
# importer's default "Position" target holds them in place. Only changed in memory.
from pxr import UsdPhysics  # noqa: E402

WHEEL_DRIVE = {"stiffness": 0.0, "damping": 1.0e4, "maxForce": 1.0e5}
for p in stage.Traverse():
    if p.GetName() in ("left_wheel", "right_wheel") and p.IsA(UsdPhysics.RevoluteJoint):
        drive = UsdPhysics.DriveAPI.Apply(p, "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(WHEEL_DRIVE["stiffness"])
        drive.CreateDampingAttr().Set(WHEEL_DRIVE["damping"])
        drive.CreateMaxForceAttr().Set(WHEEL_DRIVE["maxForce"])
        print(f"[isaac_run] velocity drive on {p.GetPath()}", flush=True)

for _ in range(10):
    app.update()
omni.timeline.get_timeline_interface().play()
print("[isaac_run] playing - drive with /cmd_vel", flush=True)

# When ros2 launch stops Isaac it signals python.sh (the bash wrapper), not this process. If the
# wrapper is gone we are orphaned (parent pid changes) -> quit, so no stale Isaac keeps running
# and publishing a second robot.
import time  # noqa: E402

parent = os.getppid()
last_check = time.monotonic()
while app.is_running():
    app.update()
    if time.monotonic() - last_check > 1.0:
        last_check = time.monotonic()
        if os.getppid() != parent:
            print("[isaac_run] launcher is gone - closing Isaac Sim", flush=True)
            break

omni.timeline.get_timeline_interface().stop()
app.close()
