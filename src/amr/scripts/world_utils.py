"""Shared helpers for the world generators (make_maze.py, make_corridor.py).

Walls get a real material (UsdPreviewSurface), not only a display colour: Isaac Sim's RTX
renderer shows materials reliably, so black walls stand out against the grey ground grid.
Every wall/box also gets a PhysX collider so the lidar and ultrasonics see it.
"""
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

WALL_BLACK = (0.02, 0.02, 0.02)
BOX_ORANGE = (0.85, 0.40, 0.10)


def material(stage, name, rgb, roughness=0.7):
    path = Sdf.Path(f"/World/Looks/{name}")
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def new_world(path, light_parent="/World/Env_Light"):
    """Create an empty Z-up, metre stage with /World as default prim and a sun light."""
    from pxr import Usd
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    light = UsdLux.DistantLight.Define(stage, light_parent)
    light.CreateIntensityAttr(3000.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 0.0, 30.0))
    return stage


def solid_box(stage, path, center, size, mat, rgb):
    """Axis-aligned box with collider and material. center/size are (x, y, z) in metres."""
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])          # fallback colour (viewers w/o materials)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
    return cube

