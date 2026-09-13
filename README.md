# abb_irb140_description

A ROS 2 (Jazzy) package for an ABB IRB140 6-DOF manipulator fitted with a
custom pneumatic parallel gripper and a wrist-mounted RGBD camera (Intel
RealSense D405). It provides the robot's URDF/xacro description, Gazebo
(gz-sim) simulation assets, RViz configuration, and the launch files, control
nodes and demo scripts needed to drive both a simulated arm and the real
robot cell through the same interfaces.

This package accompanies my dissertation project on vision-guided pick-and-
place with the IRB140.

<p align="center">
  <img src="docs/images/rviz_overview.png" alt="RViz view of the IRB140 with TF frames, the wrist RGBD point cloud and the octomap overlay" width="45%">
  <img src="docs/images/gazebo_world.png" alt="Gazebo world model: IRB140 on its podium next to the lab table" width="45%">
</p>

> **Note:** the images above are placeholders — screenshots of the RViz and
> Gazebo views (and any other renders worth including) go in `docs/images/`.

## Contents

- [Overview](#overview)
- [Requirements](#requirements)
- [Building](#building)
- [Package layout](#package-layout)
- [Simulation](#simulation-sim_robotlaunchpy)
- [Real robot](#real-robot-real_robotlaunchpy)
- [Gripper control](#gripper-control)
- [Octomapping](#octomapping)
- [Demo scripts](#demo-scripts)
- [Video demonstrations](#video-demonstrations)
- [Known limitations](#known-limitations)
- [License](#license)

## Overview

Both the simulated and real robot are driven through the same set of ROS 2
interfaces wherever possible, so code written against one transfers to the
other with minimal changes:

- `/arm_controller/follow_joint_trajectory` — `control_msgs/action/FollowJointTrajectory`
  for the 6 arm joints (`joint_1` … `joint_6`).
- `/gripper_trigger` — `std_srvs/srv/SetBool` to open/close the pneumatic
  gripper (`data: true` closes, `false` opens).
- `/camera/color/...`, `/camera/depth/...` — RealSense-style RGBD topics from
  the wrist camera, real or simulated.
- `/joint_states`, `/tf` — full robot state, merged from whichever backend is
  running underneath (`gz_ros2_control` in sim, the ABB controller bridge on
  the real robot).

## Requirements

- ROS 2 **Jazzy**
- Gazebo **Harmonic** (`ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`)
- `ros2_control` / `ros2_controllers` (`joint_state_broadcaster`,
  `joint_trajectory_controller`, `position_controllers`)
- `octomap_server` and `octomap_rviz_plugins`
- `realsense2_camera` (real robot only, for the wrist D405)
- `xacro`, `robot_state_publisher`, `rviz2`, `tf2_ros`
- `libcurl` (dev headers) — used by `pneumatic_gripper_controller` to talk to
  the ABB controller's Robot Web Services API

All ROS dependencies are declared in `package.xml`; `rosdep install` from the
workspace root will pull them in.

For the real robot, driving the arm additionally requires a running bridge
from the IRC5's ROS 1 `abb_driver` onto this host's ROS 2 graph (a
rosbridge + same-distro relay stack, kept outside this package). See
`launch/real_robot.launch.py`'s module docstring for the interfaces it's
expected to expose.

## Building

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select abb_irb140_description
source install/setup.bash
```

## Package layout

```
config/     ros2_control / controller_manager params, the RViz config,
            and the ros_gz_bridge topic mapping for the wrist RGBD camera
include/    C++ header for the pneumatic gripper controller node
launch/     sim_robot.launch.py (Gazebo) and real_robot.launch.py (hardware)
maps/       pre-built octomap octrees (.ot/.bt) for both sim and the real lab
meshes/     visual + collision meshes for the arm and the gripper
models/     Gazebo (SDF) models for the lab world: podium, table, ArUco marker
scripts/    Python nodes: pick-and-place demos, an RGBD frame-id fix-up node
src/        C++ source for the pneumatic gripper controller node
urdf/       irb140.xacro (edit this) and the gripper xacro macros it includes
worlds/     robot_lab.world, the Gazebo world used by sim_robot.launch.py
```

## Simulation (`sim_robot.launch.py`)

```bash
ros2 launch abb_irb140_description sim_robot.launch.py
```

Brings up gz-sim with `worlds/robot_lab.world`, spawns the IRB140 on its
podium, starts `gz_ros2_control`'s controller stack
(`joint_state_broadcaster`, `arm_controller`, `gripper_controller`), bridges
the wrist RGBD camera onto RealSense-style topics, and opens RViz.

| Argument             | Default                     | Description |
|----------------------|------------------------------|--------------|
| `use_rviz`           | `true`                       | Start RViz alongside the simulation |
| `use_octomap`        | `false`                      | `true`: live-map the wrist point cloud with `octomap_server`; `false`: serve the pre-built octree from `octomap_file` |
| `octomap_resolution` | `0.05`                       | Octomap voxel size in metres (`use_octomap:=true` only) |
| `octomap_file`       | `maps/robot_lab_sim.ot`      | Pre-built octree served when `use_octomap:=false` |

## Real robot (`real_robot.launch.py`)

```bash
ros2 launch abb_irb140_description real_robot.launch.py
```

Starts `robot_state_publisher`, the pneumatic gripper controller (in its
real, RWS-backed mode), RViz, and — if `use_camera:=true` (default) — the
real D405 driver plus the static transforms that graft its optical frames
onto the URDF. This launch file does **not** start `controller_manager` or
any arm controller: the arm's `/joint_states` and
`/arm_controller/follow_joint_trajectory` are expected to already be present
on the graph, published by the separate ROS 1→2 bridge mentioned above.

| Argument             | Default                     | Description |
|----------------------|------------------------------|--------------|
| `use_rviz`           | `true`                       | Start RViz alongside the real robot |
| `robot_ip`           | `192.168.125.1`              | ABB IRC5 controller IP (used for the gripper's RWS signal) |
| `use_camera`         | `true`                       | Start the real D405 RealSense driver |
| `use_octomap`        | `false`                      | Same semantics as the sim launch file |
| `octomap_resolution` | `0.05`                       | Octomap voxel size in metres (`use_octomap:=true` only) |
| `octomap_file`       | `maps/robot_lab_sim.ot`      | Pre-built octree served when `use_octomap:=false` |

### Sending a test arm goal

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      { positions: [0.0, 0.7, -0.5, 0.0, 1.4, 0.0],
        time_from_start: { sec: 2, nanosec: 0 } }
    ]
  }
}"
```

## Gripper control

The gripper is pneumatic (on/off), not a `ros2_control` joint, so it's driven
through its own `pneumatic_gripper_controller` node (`src/pneumatic_gripper_controller.cpp`)
via a single `std_srvs/srv/SetBool` service:

```bash
# Close
ros2 service call /gripper_trigger std_srvs/srv/SetBool "{data: true}"
# Open
ros2 service call /gripper_trigger std_srvs/srv/SetBool "{data: false}"
```

- **Real robot** (`sim:=false`, the node's default): sets the ABB
  controller's `closeGrippersOut` digital output signal over Robot Web
  Services (RWS) using HTTP digest auth
  (`http://<robot_ip>/rw/iosystem/signals/closeGrippersOut?action=set`,
  `lvalue=1` to close / `lvalue=0` to open), then mocks the 12 finger joints'
  `/joint_states` (nothing else publishes them on the real robot). The
  credentials used (`Default User` / `robotics`) are ABB's RobotWare factory
  defaults, not secrets specific to this cell — change them in
  `include/pneumatic_gripper_controller.hpp` if your controller's RWS user
  has been reconfigured.
- **Simulation** (`sim:=true`, set by `sim_robot.launch.py`): forwards the
  request as a `control_msgs/action/GripperCommand` goal to the Gazebo
  `gripper_controller` instead, so the same service drives either backend.

## Octomapping

Both launch files expose the same `use_octomap` switch:

- `use_octomap:=false` (default) — `octomap_server` loads and serves one of
  the pre-built octrees in `maps/` (`robot_lab_sim.ot`/`.bt` for simulation,
  `robot_lab_real.ot`/`.bt` from the real lab), latched, with no per-frame
  cost.
- `use_octomap:=true` — `octomap_server` instead builds the occupancy octree
  live from the wrist RGBD camera's point cloud.

## Demo scripts

- `scripts/irb140_pick_and_place_demo_sim.py` — minimal smoke test: sends a
  single `FollowJointTrajectory` goal and waits for the result. Useful for
  confirming the sim controller stack is up before running anything more
  involved.
- `scripts/irb140_pick_and_place_demo_real.py` — a full pick-and-place
  sequence against the real robot: open gripper → approach → pick → close
  gripper → retreat → move to box → release → return to base, driven by
  joint-space waypoints captured from the live robot. Advances each move as
  soon as `/joint_states` shows the target has been reached (rather than
  waiting out the commanded duration), so it doesn't assume anything about
  the exact motion timing of whatever is executing the trajectory.
- `scripts/fix_depth_points_frame.py` — republishes the simulated wrist
  camera's point cloud with a corrected `frame_id`, working around a gz-sensors
  upstream bug (see the script's docstring for details). Run automatically
  by `sim_robot.launch.py`; not needed on the real robot.

Both packaged as executables via `CMakeLists.txt`, so they run with:

```bash
ros2 run abb_irb140_description irb140_pick_and_place_demo_sim.py
ros2 run abb_irb140_description irb140_pick_and_place_demo_real.py
```

## Video demonstrations

- Pick-and-place on the real robot — *link TBD*
- Live octomapping in Gazebo simulation — *link TBD*
- Live octomapping on the real robot — *link TBD*

(Recorded demo videos are kept out of git due to size — add links here once
they're uploaded, e.g. to YouTube or an institutional host.)

## Known limitations

- The simulated wrist camera's point cloud is affected by an open upstream
  gz-sensors bug (frame convention mismatch on `PointCloudPacked` output);
  `scripts/fix_depth_points_frame.py` works around it. See that script's
  docstring for the full explanation and upstream issue links.
- `real_robot.launch.py` assumes a separate ROS 1↔2 bridge is already
  publishing the arm's `/joint_states` and
  `/arm_controller/follow_joint_trajectory` — it is not started by anything
  in this package.

## License

MIT — see `package.xml`.
