#!/bin/bash
# Save the current SLAM map for reuse
# Run this AFTER driving the robot around to build the map

MAP_DIR="$(dirname "$0")/src/home_robot_navigation/maps"
MAP_NAME="home_service_map"

mkdir -p "$MAP_DIR"

echo "📍 Saving map to: $MAP_DIR/$MAP_NAME"
echo "   (Make sure you have driven the robot around first!)"

source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"

ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_DIR/$MAP_NAME" \
  --ros-args -p use_sim_time:=true

if [ -f "$MAP_DIR/${MAP_NAME}.yaml" ]; then
  echo "✅ Map saved successfully!"
  echo "   Files: $MAP_DIR/${MAP_NAME}.yaml"
  echo "          $MAP_DIR/${MAP_NAME}.pgm"
  echo ""
  echo "   Next time, launch with the saved map:"
  echo "   ros2 launch home_robot_bringup full_pipeline.launch.py slam:=False"
else
  echo "❌ Map save failed. Make sure:"
  echo "   1. The pipeline is running"
  echo "   2. You drove the robot around to build a map"
  echo "   3. The /map topic is publishing"
fi
