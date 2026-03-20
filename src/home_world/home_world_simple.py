import omni
import omni.kit.commands
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.utils.stage import add_reference_to_stage

def create_box(stage, prim_path, size, position, color):
    xform_prim = stage.DefinePrim(prim_path, "Xform")
    UsdGeom.Xform(xform_prim).AddTranslateOp().Set(Gf.Vec3d(*position))
    cube = stage.DefinePrim(f"{prim_path}/Box", "Cube")
    UsdGeom.Cube(cube).GetSizeAttr().Set(1.0)
    UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(*size))
    mat = stage.DefinePrim(f"{prim_path}/Mat", "Material")
    shader = stage.DefinePrim(f"{prim_path}/Mat/Shader", "Shader")
    shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("UsdPreviewSurface")
    shader.CreateAttribute("inputs:diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    UsdPhysics.CollisionAPI.Apply(cube)
    UsdPhysics.RigidBodyAPI.Apply(xform_prim).CreateKinematicEnabledAttr().Set(True)

def build():
    stage = omni.usd.get_context().get_stage()
    for prim in stage.GetPseudoRoot().GetChildren():
        if "/World" in prim.GetPath().pathString: delete_prim(prim.GetPath().pathString)
    stage.DefinePrim("/World", "Xform")
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").CreateGravityMagnitudeAttr().Set(9.81)
    stage.DefinePrim("/World/DomeLight", "DomeLight").CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)
    create_box(stage, "/World/Floor", (6, 4, 0.01), (0, 0, 0), (0.8, 0.8, 0.7))
    create_box(stage, "/World/Table", (0.6, 0.35, 0.02), (3.5, 0, 0.4), (0.6, 0.4, 0.2))
    create_box(stage, "/World/RedCube", (0.025, 0.025, 0.025), (3.5, 0.1, 0.44), (1, 0, 0))
    add_reference_to_stage("omniverse://localhost/Isaac/Robots/Franka/franka.usd", "/World/Robot")
    print("✅ Scene built successfully!")

build()
