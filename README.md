# HX35HM + SO101 Control Stack

Control-side workspace for the HX35HM + SO101 arm setup.

This repo focuses on:

- HX35HM bridge and servo control
- ROS 2 bringup and controllers
- MoveIt planning and execution
- teleop
- grasping and calibration tooling

## Quick Start

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_ws/src/install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_hx35hm:=true \
  use_cameras:=false \
  use_rviz:=true \
  use_joint_gui:=false \
  use_aruco_detector:=false \
  use_red_detector:=false
```

If you only want the shortest operator guide, open:

- [HX35HM_SO101_MoveIt规划控制启动流程.md](src/HX35HM_SO101_MoveIt规划控制启动流程.md)

## What Is Inside

- `src/so101_hx35hm_bridge` - HX35HM serial bridge, ArUco detector, red block detector, assembly helpers
- `src/so101-ros-physical-ai/so101_bringup` - top-level launch files, controller configs, camera configs, RViz configs
- `src/so101-ros-physical-ai/so101_description` - URDF/Xacro, meshes, ros2_control description
- `src/so101-ros-physical-ai/so101_moveit_config` - MoveIt planning config, SRDF, kinematics, controllers
- `src/so101-ros-physical-ai/so101_teleop` - leader-to-follower teleop
- `src/so101-ros-physical-ai/so101_grasping` - visual grasping demo nodes and launch files
- `src/so101-ros-physical-ai/tools` - hand-eye and camera calibration scripts

## Minimal Workflow

1. Build the workspace.
2. Launch `follower_hx35hm_moveit.launch.py`.
3. Verify `move_group`, `hx35hm_bridge`, and `arm_trajectory_controller` are up.
4. Use RViz to plan and execute `rest`, `zero`, or `extended`.
5. Read the MoveIt workflow doc if planning or execution fails.

## Notes

- This is a control-focused export of the larger ROS workspace.
- Before using real hardware, make sure the arm calibration and device naming are already correct.
- If you need teleop only, the launch entry is `ros2 launch so101_bringup teleop.launch.py`.
