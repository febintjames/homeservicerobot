# Isaac Sim Setup Guide — Home Service Robot

Step-by-step guide to run your homeservicerobot project in **NVIDIA Isaac Sim** 
(Physical AI) instead of Gazebo, using your T4 cloud VM connected via WebRTC.

---

## Prerequisites

| Item | Status |
|------|--------|
| Isaac Sim 5.1 running on T4 cloud VM | ✅ Already done |
| WebRTC client connected locally | ✅ Already done |
| ROS2 Humble installed on VM | ✅ Already done |
| homeservicerobot ROS2 workspace built | ✅ Already done |

---

## Step 1 — Open Isaac Sim via WebRTC

1. Open your WebRTC client / browser stream to the VM
2. You should see the Isaac Sim UI (viewport, menus, toolbar)

---

## Step 2 — Load the Home World Script

1. In Isaac Sim, go to **Window → Script Editor**
2. Click **File → Open** and navigate to:
   ```
   ~/ros2_ws/src/homeservicerobot/src/home_world/home_world_isaac.py
   ```
   > **OR** copy-paste the contents of `home_world_isaac.py` directly into the editor
3. Click the **▶ Run** button (top-right of Script Editor)

**You should see in the Script Editor output:**

```
============================================================
  Isaac Sim — Home Service Robot World
  Building scene from home.world (Gazebo SDF equivalent)
============================================================

[Isaac Home World] Setting up scene geometry...
[Isaac Home World] ✅ Scene geometry built successfully!
[Isaac Home World] Loading TIAGo robot...
[Isaac Home World] ✅ TIAGo robot imported from URDF
[Isaac Home World] Setting up ROS2 bridge...
[ROS2 Bridge] ✅ Action Graph created at: /World/ROS2ActionGraph

============================================================
  ✅ Home World Ready!
  Press the PLAY button (▶) to start simulation + ROS2 bridge
============================================================
```

**You should see in the Viewport:** floor, walls, kitchen with 3 coloured cubes (red/green/blue) on the table, sofa and coffee table in living room, TIAGo robot at origin.

---

## Step 3 — Handle TIAGo URDF Path

If the script prints `⚠️ URDF not found`, you need to set the correct path.

**Find the URDF on your VM:**
```bash
find ~/ros2_ws -name "tiago.urdf" 2>/dev/null
```

**Then edit `home_world_isaac.py` line ~170:**
```python
TIAGO_URDF_PATH = "/home/user/ros2_ws/install/tiago_description/share/tiago_description/robots/tiago.urdf"
# Change to your actual path ↑
```

Alternatively, use the **Isaac Sim URDF Importer** tool directly:
1. **File → Import** → select `tiago.urdf`
2. The robot will appear in the viewport
3. Rename its stage prim to `/World/TiagoRobot`

---

## Step 4 — Press PLAY

Click the **▶ Play** button in the Isaac Sim toolbar.

The physics simulation and ROS2 bridge start simultaneously.

---

## Step 5 — Verify ROS2 Topics (SSH into VM)

Open a terminal on the VM (or SSH in from your local machine):

```bash
# Source your ROS2 workspace
source ~/ros2_ws/install/setup.bash

# Check that Isaac Sim bridge topics are publishing
ros2 topic list
```

**Expected output includes:**
```
/joint_states
/mobile_base_controller/odom
/head_front_camera/rgb/image_raw
/head_front_camera/depth/image_raw
/scan_raw
/tf
/tf_static
/cmd_vel
```

**Quick sanity check:**
```bash
# Verify LiDAR is working
ros2 topic echo /scan_raw --once

# Verify camera is working
ros2 topic hz /head_front_camera/rgb/image_raw
```

---

## Step 6 — Launch the ROS2 Pipeline

> ⚠️ Do **NOT** use `full_pipeline.launch.py` — that launches Gazebo.  
> Use the new `isaac_pipeline.launch.py` instead.

```bash
source ~/ros2_ws/install/setup.bash

# Launch with SLAM (no pre-built map needed)
ros2 launch home_robot_bringup isaac_pipeline.launch.py target_color:=red

# OR: Launch with a pre-built map
ros2 launch home_robot_bringup isaac_pipeline.launch.py \
    slam:=False \
    map:=/home/user/my_map.yaml \
    target_color:=red
```

Wait ~20 seconds for all nodes to start (Nav2, MoveIt2, task_coordinator).

---

## Step 7 — Trigger the Pick-and-Deliver Task

```bash
# In a new terminal
source ~/ros2_ws/install/setup.bash

# Send the bring_food service call
ros2 service call /bring_food std_srvs/srv/Trigger "{}"
```

**Watch in Isaac Sim viewport:**
1. 🤖 TIAGo navigates to the kitchen table
2. 🦾 Arm reaches down, gripper closes on the cube
3. 🚶 Robot navigates to the living room (delivery target: x=-2.5, y=0)
4. 📦 Arm lowers, gripper releases the cube

---

## Troubleshooting

### ROS2 topics not appearing after pressing Play

```bash
# Check if ROS2 bridge extension is enabled in Isaac Sim
# Window → Extensions → search "ROS2 Bridge" → verify it's toggled ON
# Then restart the script
```

### TIAGo not moving (cmd_vel ignored)

Make sure the `SubscribeTwist` node in the Action Graph is connected to `TiagoRobot`.
Open **Window → Visual Scripting → Action Graph** → check `/World/ROS2ActionGraph`.

### Nav2 can't connect to controller

Verify the controller topics by checking:
```bash
ros2 action list
# Should show:
# /arm_controller/follow_joint_trajectory
# /gripper_controller/follow_joint_trajectory
# /mobile_base_controller/...
```

### Cube falls through the table

The cube physics requires the table surface to have collision enabled.
The script sets this up automatically. If it fails, in Isaac Sim:
1. Select `/World/KitchenTable/Top` in Stage
2. Add component: **Physics → Rigid Body** (kinematic)
3. Add component: **Physics → Collider**

### Camera stream blank

Check the camera prim path. If TIAGo URDF was renamed on import, update in `home_world_isaac.py`:
```python
"PublishRGBCamera.inputs:targetPrim":  "/World/YOUR_ROBOT_PRIM/head_front_camera_link"
```

---

## File Reference

| File | Purpose |
|------|---------|
| `src/home_world/home_world_isaac.py` | **Run this in Isaac Sim Script Editor** |
| `src/home_world/ros2_bridge_config.yaml` | Reference for all topic/prim mappings |
| `src/home_robot_bringup/launch/isaac_pipeline.launch.py` | **ROS2 launch command (replaces Gazebo)** |
| `src/home_robot_bringup/launch/full_pipeline.launch.py` | Old Gazebo launch (not used with Isaac Sim) |
| `src/task_coordinator/task_coordinator/task_coordinator_node.py` | **Unchanged** — works with both Gazebo and Isaac Sim |
| `src/task_coordinator/task_coordinator/food_detector.py` | **Unchanged** — HSV detection works on Isaac Sim camera |
