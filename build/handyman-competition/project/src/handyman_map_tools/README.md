# handyman_map_tools

Validate a Unity semantic export:

```bash
ros2 run handyman_map_tools validate_semantic_map \
  /mnt/d/handyman/exports/LayoutA/semantic_map.json
```

Publish rooms, destinations, object-spawn candidates, and the initial robot pose for RViz inspection:

```bash
ros2 run handyman_map_tools semantic_marker_node --ros-args \
  -p input:=/mnt/d/handyman/exports/LayoutA/semantic_map.json \
  -p frame_id:=map
```

Add a `MarkerArray` display for `/semantic_map_markers` in RViz. The publisher uses transient-local durability, so RViz may be opened after the command runs.

Import the four current competition layouts into the navigation package:

```bash
ros2 run handyman_map_tools import_unity_maps \
  /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/maps/export \
  ~/RoboCup-Japan-Open-2026/src/handyman_rebuild_ros2
```

The command copies each map into the ROS package, replaces the four generated
environment YAML files, derives collision-safe room search points and multiple
approach poses for every destination, and writes `maps/import_report.json`.
Disconnected room components are reported explicitly instead of silently
producing an unreachable Nav2 goal.

For competition-day import, run the workspace helper after exporting from
Unity. It also rebuilds the packages and runs the complete regression suite:

```bash
cd ~/RoboCup-Japan-Open-2026
./import_handyman_maps.sh /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/maps/export
```
