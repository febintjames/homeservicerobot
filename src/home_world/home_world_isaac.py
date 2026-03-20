#!/usr/bin/env python3
"""
Home World - Isaac Sim Script
=====================================
Recreates the 'home.world' Gazebo environment inside NVIDIA Isaac Sim
using the omni.isaac.core Python API and PhysX physics.

HOW TO USE:
  1. Open Isaac Sim via your WebRTC browser connection
  2. Go to: Window → Script Editor
  3. Paste this entire script (or File → Open this file)
  4. Click "Run" (triangle button)
  5. The home scene will build and the ROS2 bridge will start automatically

SCENE LAYOUT (matches home.world exactly):
  - Floor: 12m x 8m
  - Outer walls: North/South/East/West
  - Divider wall with doorway (Kitchen | Living Room)
  - Kitchen area (x > 1): table with 3 colored cubes + counter
  - Living room (x < 1): sofa + coffee table
  - TIAGo robot spawned at (0, 0, 0) — imports from URDF

ROS2 Topics bridged:
  /cmd_vel                          → robot base velocity
  /joint_states                     → all joint positions
  /head_front_camera/rgb/image_raw  → RGB camera
  /head_front_camera/depth          → depth camera
  /scan_raw                         → LiDAR
  /mobile_base_controller/odom      → odometry
  /tf                               → transforms

Author: Febin TJ  |  Isaac Sim 5.1 + ROS2 Humble
"""

import omni
import omni.kit.commands
import omni.usd
from omni.isaac.core import SimulationApp

# ─── If running from Script Editor (SimulationApp already started), 
#     skip the SimulationApp block below. Isaac Sim's Script Editor 
#     executes inside the already-running app context.
try:
    simulation_app
    RUNNING_IN_SCRIPT_EDITOR = True
except NameError:
    RUNNING_IN_SCRIPT_EDITOR = False

if not RUNNING_IN_SCRIPT_EDITOR:
    simulation_app = SimulationApp({
        "headless": False,
        "width": 1920,
        "height": 1080,
    })

# ─── Core imports (available after SimulationApp is started) ─────────────────
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim, delete_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
import omni.isaac.core.utils.prims as prim_utils

# ─── ROS2 Bridge imports ─────────────────────────────────────────────────────
try:
    import omni.isaac.ros2_bridge as ros2_bridge
    ROS2_AVAILABLE = True
except ImportError:
    print("[WARN] omni.isaac.ros2_bridge not found — ROS2 bridge will not start.")
    ROS2_AVAILABLE = False

# =============================================================================
#                         HELPER FUNCTIONS
# =============================================================================

def create_box(
    stage,
    prim_path: str,
    size: tuple,        # (x, y, z) half-extents in metres
    position: tuple,   # (x, y, z) world position
    color: tuple,      # (r, g, b) 0-1
    is_static: bool = True,
    mass: float = None,
):
    """Create a box primitive on the USD stage with optional physics."""
    xform_prim = stage.DefinePrim(prim_path, "Xform")
    UsdGeom.Xform(xform_prim).AddTranslateOp().Set(Gf.Vec3d(*position))

    cube = stage.DefinePrim(f"{prim_path}/Cube", "Cube")
    UsdGeom.Cube(cube).GetSizeAttr().Set(1.0)

    # Scale the unit cube to desired size
    xform = UsdGeom.Xformable(cube)
    scale_op = xform.AddScaleOp()
    scale_op.Set(Gf.Vec3f(*size))

    # Material / colour
    mat_path = f"{prim_path}/Material"
    mat = stage.DefinePrim(mat_path, "Material")
    shader_path = f"{mat_path}/Shader"
    shader = stage.DefinePrim(shader_path, "Shader")
    shader.CreateAttribute("info:id", Sdf.ValueTypeNames.Token).Set("UsdPreviewSurface")
    shader.CreateAttribute("inputs:diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateAttribute("inputs:roughness", Sdf.ValueTypeNames.Float).Set(0.6)

    mat_binding = UsdGeom.Imageable(cube).GetPrim()
    UsdPhysics.CollisionAPI.Apply(cube)

    # Physics
    if is_static:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(xform_prim)
        rigid_api.CreateKinematicEnabledAttr().Set(True)
    else:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(xform_prim)
        rigid_api.CreateKinematicEnabledAttr().Set(False)
        if mass is not None:
            mass_api = UsdPhysics.MassAPI.Apply(xform_prim)
            mass_api.CreateMassAttr().Set(mass)

    return xform_prim


# =============================================================================
#                         WORLD SETUP
# =============================================================================

def build_home_world():
    """Build the complete home environment on the current USD stage."""

    # Get the active stage
    stage = omni.usd.get_context().get_stage()

    # Clear existing prims (except /World)
    for prim in stage.GetPseudoRoot().GetChildren():
        if prim.GetPath().pathString not in ["/World", "/OmniverseKit_Persp",
                                              "/OmniverseKit_Front", "/OmniverseKit_Top",
                                              "/OmniverseKit_Right", "/Render"]:
            delete_prim(prim.GetPath().pathString)

    world_path = "/World"
    if not stage.GetPrimAtPath(world_path):
        stage.DefinePrim(world_path, "Xform")

    # ── Physics scene ────────────────────────────────────────────────────────
    physics_scene_path = "/World/PhysicsScene"
    physics_scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_path))
    physx_scene.CreateEnableCCDAttr().Set(True)

    # ── Lighting ─────────────────────────────────────────────────────────────
    dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
    dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)

    distant_light = stage.DefinePrim("/World/SunLight", "DistantLight")
    UsdGeom.Xform(distant_light).AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 135))
    distant_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(2000.0)
    distant_light.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.98, 0.9))

    print("[Isaac Home World] Setting up scene geometry...")

    # ==========================================================================
    #  FLOOR  — 12m x 8m x 0.02m  at origin
    # ==========================================================================
    create_box(stage, "/World/Floor",
               size=(6.0, 4.0, 0.01),        # half-extents: 12/2, 8/2, 0.02/2
               position=(0, 0, 0),
               color=(0.85, 0.82, 0.75),
               is_static=True)

    # ==========================================================================
    #  OUTER WALLS
    # ==========================================================================
    # North wall: 12 × 0.15 × 2.5  at y=4
    create_box(stage, "/World/WallNorth",
               size=(6.0, 0.075, 1.25),
               position=(0, 4, 1.25),
               color=(0.9, 0.9, 0.88), is_static=True)

    # South wall: at y=-4
    create_box(stage, "/World/WallSouth",
               size=(6.0, 0.075, 1.25),
               position=(0, -4, 1.25),
               color=(0.9, 0.9, 0.88), is_static=True)

    # East wall: 0.15 × 8 × 2.5  at x=6
    create_box(stage, "/World/WallEast",
               size=(0.075, 4.0, 1.25),
               position=(6, 0, 1.25),
               color=(0.9, 0.9, 0.88), is_static=True)

    # West wall: at x=-6
    create_box(stage, "/World/WallWest",
               size=(0.075, 4.0, 1.25),
               position=(-6, 0, 1.25),
               color=(0.9, 0.9, 0.88), is_static=True)

    # ==========================================================================
    #  DIVIDER WALL (Kitchen | Living Room) with doorway at y=0 ±0.75
    # ==========================================================================
    # Left section: x=1, y from -4 to -1.5
    create_box(stage, "/World/DividerLeft",
               size=(0.075, 1.25, 1.25),
               position=(1, -2.75, 1.25),
               color=(0.85, 0.85, 0.82), is_static=True)

    # Right section: x=1, y from 1.5 to 4
    create_box(stage, "/World/DividerRight",
               size=(0.075, 1.25, 1.25),
               position=(1, 2.75, 1.25),
               color=(0.85, 0.85, 0.82), is_static=True)

    # Top lintel above doorway
    create_box(stage, "/World/DividerTop",
               size=(0.075, 1.5, 0.25),
               position=(1, 0, 2.25),
               color=(0.85, 0.85, 0.82), is_static=True)

    # ==========================================================================
    #  KITCHEN AREA  (x > 1)
    # ==========================================================================
    # Kitchen Table — tabletop
    create_box(stage, "/World/KitchenTable/Top",
               size=(0.6, 0.35, 0.02),
               position=(3.5, 0, 0.395),
               color=(0.6, 0.4, 0.25), is_static=True)
    # Legs
    for name, pos in [("Leg1", (4.0, 0.25, 0.185)),
                       ("Leg2", (3.0, 0.25, 0.185)),
                       ("Leg3", (4.0, -0.25, 0.185)),
                       ("Leg4", (3.0, -0.25, 0.185))]:
        create_box(stage, f"/World/KitchenTable/{name}",
                   size=(0.025, 0.025, 0.175),
                   position=pos,
                   color=(0.5, 0.35, 0.2), is_static=True)

    # Kitchen Counter along east wall
    create_box(stage, "/World/KitchenCounter",
               size=(0.3, 2.0, 0.45),
               position=(5.6, 0, 0.45),
               color=(0.65, 0.65, 0.65), is_static=True)

    # ==========================================================================
    #  COLORED CUBES  (dynamic — can be picked up)
    #  Placed on kitchen table at height z=0.43 (table top 0.415 + half cube 0.025)
    # ==========================================================================
    # Red cube
    create_box(stage, "/World/RedCube",
               size=(0.025, 0.025, 0.025),
               position=(3.5, 0.1, 0.44),
               color=(1.0, 0.15, 0.15),
               is_static=False, mass=0.05)

    # Green cube
    create_box(stage, "/World/GreenCube",
               size=(0.025, 0.025, 0.025),
               position=(3.5, 0.0, 0.44),
               color=(0.15, 1.0, 0.15),
               is_static=False, mass=0.05)

    # Blue cube
    create_box(stage, "/World/BlueCube",
               size=(0.025, 0.025, 0.025),
               position=(3.5, -0.1, 0.44),
               color=(0.15, 0.15, 1.0),
               is_static=False, mass=0.05)

    # ==========================================================================
    #  LIVING ROOM AREA  (x < 1)
    # ==========================================================================
    # Sofa — seat
    create_box(stage, "/World/Sofa/Seat",
               size=(0.35, 0.9, 0.22),
               position=(-4.5, 0, 0.23),
               color=(0.3, 0.4, 0.6), is_static=True)
    # Sofa — backrest
    create_box(stage, "/World/Sofa/Back",
               size=(0.06, 0.9, 0.325),
               position=(-4.8, 0, 0.565),
               color=(0.3, 0.4, 0.6), is_static=True)

    # Coffee table — top
    create_box(stage, "/World/CoffeeTable/Top",
               size=(0.3, 0.5, 0.02),
               position=(-3.2, 0, 0.21),
               color=(0.6, 0.4, 0.25), is_static=True)

    print("[Isaac Home World] ✅ Scene geometry built successfully!")
    print("[Isaac Home World] Rooms: Kitchen (x>1) + Living Room (x<1)")
    print("[Isaac Home World] Objects: Red/Green/Blue cubes on kitchen table")

    return stage


# =============================================================================
#                         ROBOT IMPORT  (TIAGo URDF)
# =============================================================================

def load_tiago_robot():
    """
    Import TIAGo robot into Isaac Sim.
    
    OPTION 1 (Recommended): Import via URDF Importer
      - Make sure the TIAGo URDF is accessible on the VM filesystem
      - Adjust TIAGO_URDF_PATH below to the correct path
      
    OPTION 2: Use Isaac Sim's built-in Franka as a placeholder
      - Set USE_PLACEHOLDER = True
    """
    USE_PLACEHOLDER = False  # ← Set True to use Franka placeholder
    
    # Path to TIAGo URDF on the cloud VM
    # Typically in your ROS2 workspace install directory
    TIAGO_URDF_PATH = "/home/user/ros2_ws/install/tiago_description/share/tiago_description/robots/tiago.urdf"

    stage = omni.usd.get_context().get_stage()

    if USE_PLACEHOLDER:
        # ── Franka placeholder ────────────────────────────────────────────
        franka_usd = "/Isaac/Robots/Franka/franka.usd"
        robot_prim_path = "/World/TiagoPlaceholder"
        add_reference_to_stage(
            usd_path=f"omniverse://localhost{franka_usd}",
            prim_path=robot_prim_path
        )
        prim = stage.GetPrimAtPath(robot_prim_path)
        UsdGeom.Xform(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        print("[Isaac Home World] ⚠️  Using Franka placeholder robot at (0,0,0)")
        print("[Isaac Home World]    Replace with TIAGo URDF when ready.")
    else:
        # ── TIAGo URDF import ─────────────────────────────────────────────
        import omni.kit.commands
        import os

        if not os.path.exists(TIAGO_URDF_PATH):
            print(f"[Isaac Home World] ⚠️  URDF not found at: {TIAGO_URDF_PATH}")
            print("[Isaac Home World]    Edit TIAGO_URDF_PATH in load_tiago_robot()")
            print("[Isaac Home World]    OR set USE_PLACEHOLDER = True")
            return None

        robot_prim_path = "/World/TiagoRobot"
        status, import_config = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=TIAGO_URDF_PATH,
            import_config=None,
        )
        if status:
            # Move robot to spawn pose (centre of house, facing kitchen)
            prim = stage.GetPrimAtPath(robot_prim_path)
            if prim:
                UsdGeom.Xform(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.1))
            print(f"[Isaac Home World] ✅ TIAGo robot imported from URDF")
            print(f"[Isaac Home World]    Prim path: {robot_prim_path}")
        else:
            print("[Isaac Home World] ❌ URDF import failed. Check the path and try again.")

    return robot_prim_path


# =============================================================================
#                         ROS2 BRIDGE SETUP
# =============================================================================

def setup_ros2_bridge():
    """
    Enable the Isaac Sim ROS2 Bridge and set up all topic publishers/subscribers.
    
    Topics match the existing homeservicerobot ROS2 stack.
    Run this AFTER building the world and loading the robot.
    """
    if not ROS2_AVAILABLE:
        print("[ROS2 Bridge] ❌ ros2_bridge extension not available.")
        print("[ROS2 Bridge]    Enable it via: Window → Extensions → search 'ROS2 Bridge' → Enable")
        return

    # Enable the ROS2 bridge extension
    manager = omni.kit.app.get_app().get_extension_manager()
    bridge_ext = "omni.isaac.ros2_bridge"
    if not manager.is_extension_enabled(bridge_ext):
        manager.set_extension_enabled_immediate(bridge_ext, True)
        print(f"[ROS2 Bridge] ✅ Enabled extension: {bridge_ext}")

    stage = omni.usd.get_context().get_stage()

    # ── Action Graph prim for ROS2 OmniGraph nodes ────────────────────────────
    graph_path = "/World/ROS2ActionGraph"
    try:
        import omni.graph.core as og

        (graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick",          "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context",             "omni.isaac.ros2_bridge.ROS2Context"),
                    # TF publisher
                    ("PublishTF",               "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    # Joint state publisher  
                    ("PublishJointState",       "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    # Odometry publisher
                    ("PublishOdometry",         "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    # Twist subscriber (cmd_vel)
                    ("SubscribeTwist",          "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                    # Camera RGB publisher
                    ("PublishRGBCamera",        "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    # LiDAR publisher
                    ("PublishLidar",            "omni.isaac.ros2_bridge.ROS2LidarHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick",   "PublishTF.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",   "PublishOdometry.inputs:execIn"),
                    ("ROS2Context.outputs:context",   "PublishTF.inputs:context"),
                    ("ROS2Context.outputs:context",   "PublishJointState.inputs:context"),
                    ("ROS2Context.outputs:context",   "PublishOdometry.inputs:context"),
                    ("ROS2Context.outputs:context",   "SubscribeTwist.inputs:context"),
                    ("ROS2Context.outputs:context",   "PublishRGBCamera.inputs:context"),
                    ("ROS2Context.outputs:context",   "PublishLidar.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Joint state topic
                    ("PublishJointState.inputs:topicName",    "/joint_states"),
                    ("PublishJointState.inputs:targetPrim",   "/World/TiagoRobot"),
                    # Odometry topic
                    ("PublishOdometry.inputs:topicName",      "/mobile_base_controller/odom"),
                    ("PublishOdometry.inputs:chassisPrim",    "/World/TiagoRobot"),
                    # cmd_vel subscriber
                    ("SubscribeTwist.inputs:topicName",       "/cmd_vel"),
                    # Camera
                    ("PublishRGBCamera.inputs:topicName",     "/head_front_camera/rgb/image_raw"),
                    ("PublishRGBCamera.inputs:type",          "rgb"),
                    # LiDAR
                    ("PublishLidar.inputs:topicName",         "/scan_raw"),
                ],
            }
        )
        print("[ROS2 Bridge] ✅ Action Graph created at:", graph_path)
    except Exception as e:
        print(f"[ROS2 Bridge] ⚠️  OmniGraph setup error: {e}")
        print("[ROS2 Bridge]    You can also configure topics manually via:")
        print("[ROS2 Bridge]    Isaac Sim menu → ROS2 → Configure Bridge")

    print("\n[ROS2 Bridge] Topic Mappings:")
    print("  /joint_states                        ← TiagoRobot joints")
    print("  /mobile_base_controller/odom         ← base odometry")
    print("  /cmd_vel                             → base velocity commands")
    print("  /head_front_camera/rgb/image_raw     ← RGB camera")
    print("  /scan_raw                            ← LiDAR scan")
    print("  /tf                                  ← transform tree")


# =============================================================================
#                         MAIN — RUN EVERYTHING
# =============================================================================

print("\n" + "="*60)
print("  Isaac Sim — Home Service Robot World")
print("  Building scene from home.world (Gazebo SDF equivalent)")
print("="*60 + "\n")

# Step 1: Build the world geometry
stage = build_home_world()

# Step 2: Load the TIAGo robot
print("\n[Isaac Home World] Loading TIAGo robot...")
robot_prim = load_tiago_robot()

# Step 3: Start ROS2 bridge
print("\n[Isaac Home World] Setting up ROS2 bridge...")
setup_ros2_bridge()

print("\n" + "="*60)
print("  ✅ Home World Ready!")
print("  Press the PLAY button (▶) to start simulation + ROS2 bridge")
print("  Then on your ROS2 terminal:")
print("    ros2 topic list   # verify topics appear")
print("  Then launch the pipeline:")
print("    ros2 launch home_robot_bringup isaac_pipeline.launch.py")
print("="*60 + "\n")
