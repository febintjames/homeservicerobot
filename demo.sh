#!/bin/bash
# ============================================================
# Cloud-Based Autonomous Home Service Robot - Demo Script
# ============================================================
# This script launches the full pipeline and triggers the
# food delivery task after the system initializes.
#
# Usage: ./demo.sh
#
# Author: Febin TJ
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "============================================================"
echo "  CLOUD-BASED AUTONOMOUS HOME SERVICE ROBOT"
echo "  Demo Script"
echo "============================================================"
echo ""

# Source ROS2 and workspace
echo "[1/4] Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

echo "[2/4] Sourcing workspace..."
source "$WS_DIR/install/setup.bash"

# Set Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "[3/4] Launching full pipeline..."
echo "   This will start: Gazebo + TIAGo + Nav2 + MoveIt2 + Task Coordinator"
echo ""
echo "   Wait for all systems to initialize (~30 seconds), then run:"
echo ""
echo "   ros2 run task_coordinator bring_food"
echo ""
echo "   OR:"
echo ""
echo '   ros2 service call /bring_food std_srvs/srv/Trigger "{}"'
echo ""
echo "============================================================"

ros2 launch home_robot_bringup full_pipeline.launch.py
