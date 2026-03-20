# Implementation Plan: Migrate Cube Picking to Isaac Sim on GCP

This plan outlines the steps to transition your TIAGo simulation from Gazebo to NVIDIA Isaac Sim on Google Cloud. Isaac Sim provides more stable physics (PhysX 5) for contact-rich tasks like picking cubes, which should resolve the "not picking" issue in Gazebo.

## Proposed Changes

We will maintain your existing ROS 2 `task_coordinator` logic but swap the Gazebo simulation environment for an Isaac Sim instance running on GCP.

### 1. Cloud Infrastructure (GCP)
- **Instance**: Create a Compute Engine instance with an NVIDIA T4 (standard) or L4 (next-gen) GPU.
- **OS**: Ubuntu 22.04 LTS with NVIDIA Drivers (535+).
- **Tool**: Use [Isaac Automator](https://github.com/NVIDIA-Omniverse/isaac-automator) for a one-script setup of Isaac Sim on GCP.

### 2. Isaac Sim Setup
- **Robot**: Import the TIAGo USD model. You can use the `tiago_isaac` repository from Bonn-Ais or convert your current URDF using the Isaac Sim URDF Importer.
- **World**: Recreate your `home_service.world` in Isaac Sim by importing the same furniture/objects as USD files.

### 3. ROS 2 Bridge Integration
- **Distribution**: Ensure [Isaac Sim ROS 2 Bridge](https://docs.omniverse.nvidia.com/isaacsim/latest/features/ros2_bridge/index.html) is enabled (Humble distribution).
- **Communication**: Map the following Isaac Sim components to ROS 2 topics/actions:
    - `/arm_controller/follow_joint_trajectory` (Action)
    - `/gripper_controller/follow_joint_trajectory` (Action)
    - `/gripper_wrist_camera/image_raw` (Topic/Sensor)
    - `/detected_food_pose` (Maintain current vision logic or use Isaac Sim's ground truth for debugging)

### 4. Adaptation of `task_coordinator_node.py`
- No major code logic changes required, but we may need to adjust:
    - `gripper_close` positions (Isaac Sim units for gripper joints).
    - `moveit2` velocity/acceleration limits for smoother motion in PhysX.

## Verification Plan

### Automated Verification
- Run the `task_coordinator` against the Isaac Sim bridge and monitor the `GRASP_CLOSE` and `VERIFY_GRASP` states.
- Log the contact forces in Isaac Sim to ensure a stable squeeze.

### Manual Verification
1. **Remote Streaming**: Connect to the GCP instance using the Omniverse Streaming Client (Windows/Linux).
2. **Visual Check**: Observe the TIAGo robot's approach and grasp in the high-fidelity Isaac Sim viewport.
3. **Success Rate**: Perform 10 consecutive pick-and-place trials to confirm reliability over the previous Gazebo setup.
