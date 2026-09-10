# abb_driver (ROS 1) → ROS 2 Jazzy (host)

Two containers bridge the ABB IRC5 into the host's native ROS 2 Jazzy graph:

| container | base | role |
| --- | --- | --- |
| `abb-ros1` | ROS 1 Noetic | `roscore` + `abb_driver` (TCP to the IRC5) + **`rosbridge_server`** — the ROS 1 graph exposed on `ws://localhost:9090` |
| `abb-relay` | ROS 2 Jazzy | `roslibpy` client that consumes the WebSocket and **re-publishes native Jazzy topics** on the host graph |

### Why not `ros1_bridge`

`ros1_bridge` has to be compiled against a ROS 1 **and** a ROS 2 in one image. The
only ROS 2 that `apt`-installs next to Noetic is **Foxy**, and Foxy's Fast-DDS 2.1
cannot exchange discovery/data with Jazzy's Fast-DDS 2.14 — the bridge crashes
(`deserialize_change` → `std::bad_alloc`) as soon as a Jazzy node joins the
domain. Building `ros1_bridge` on Jazzy with Noetic from RoboStack is possible but
RoboStack has no `abb_driver`/`industrial_core`, so everything would be a
from-source conda build. `rosbridge` + a same-distro relay avoids DDS entirely for
the ROS 1↔ROS 2 hop and is what this setup does.

## Build & run

```bash
cd docker/abb-ros1-bridge
./run.sh build      # ~3-5 min, no big compiles
./run.sh up         # starts abb-ros1 then abb-relay
./run.sh logs       # follow both
./run.sh down       # stop + remove
```

`docker-compose.yml` is equivalent if you have the compose plugin
(`sudo apt install docker-compose-plugin`; then `docker compose up`).

Controller address / linkage:

```bash
ROBOT_IP=192.168.125.1 J23_COUPLED=false ./run.sh up
```

## Verify on the host

Uses your normal environment — the host runs **CycloneDDS**
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) and the relay is configured to match, so
no overrides are needed:

```bash
ros2 topic echo /joint_states          # sensor_msgs/msg/JointState, ~9.4 Hz, live from the IRC5
ros2 topic hz /joint_states
```

## Interfaces on the host

| host topic / interface | type | direction | notes |
| --- | --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | robot → ROS 2 | **primary**, ~9.4 Hz |
| `/feedback_states` | `control_msgs/.../FollowJointTrajectory_Feedback` | robot → ROS 2 | trajectory-execution feedback |
| `/robot_status/json` | `std_msgs/msg/String` | robot → ROS 2 | full `industrial_msgs/RobotStatus` as JSON |
| `/robot_status/in_motion` | `std_msgs/msg/Bool` | robot → ROS 2 | decoded |
| `/robot_status/e_stopped` | `std_msgs/msg/Bool` | robot → ROS 2 | decoded |
| `/robot_status/motion_possible` | `std_msgs/msg/Bool` | robot → ROS 2 | **false ⇒ motion is blocked** (motors off / protective stop) |
| `/robot_status/drives_powered` | `std_msgs/msg/Bool` | robot → ROS 2 | decoded |
| `/robot_status/error_code` | `std_msgs/msg/Int32` | robot → ROS 2 | decoded |
| `/joint_path_command` | `trajectory_msgs/msg/JointTrajectory` | ROS 2 → robot | direct to ROS 1 `motion_download_interface`; scaled by `SPEED_SCALE` |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | ROS 2 → robot | **for MoveIt**; publishes to `/joint_path_command`, tracks completion from `/joint_states`+`/robot_status` |
| `/stop_motion` | `std_srvs/srv/Trigger` | ROS 2 → robot | calls ROS 1 `industrial_msgs/StopMotion` |

Notes:

- **Motion requires `motion_possible: true`.** The classic `abb_driver` has no
  ROS service to power the drives — press **Motors On** at the controller/pendant
  (AUTO mode, RAPID ROS server task running). The action rejects immediately with
  a clear `error_string` when `motion_possible` is false.
- The **action does not proxy the ROS 1 `/joint_trajectory_action`** — that node
  prefixes an empty (STOP) `/joint_path_command` that wedges the ABB driver.
- **`SPEED_SCALE`** (default 2.0) only affects the `/joint_path_command` **topic**
  path. The **action** path uses `ACTION_SPEED_SCALE` (default 1.0) — leave it at
  1.0 and control MoveIt speed with MoveIt's own velocity/accel scaling factors.
- **`industrial_msgs` has no ROS 2 build** → `/robot_status` is JSON + decoded
  scalars, not the native message.
- **TF** is not produced by `abb_driver`. Run `robot_state_publisher` on the host
  with the `abb_irb140_description` URDF, fed by the relayed `/joint_states`.

## MoveIt 2 integration

MoveIt executes trajectories through a `FollowJointTrajectory` action. Point your
`moveit_controllers.yaml` at the relay's action:

```yaml
moveit_simple_controller_manager:
  controller_names: [arm_controller]
  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory        # -> /arm_controller/follow_joint_trajectory
    default: true
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
```

(Change `FJT_ACTION_NAME` on `abb-relay` if your controller name differs.) MoveIt
also consumes the relayed `/joint_states`. Run `robot_state_publisher` for TF.

## Configuration (env vars)

`abb-ros1`: `ROBOT_IP`, `J23_COUPLED`, `ROSBRIDGE_PORT` (9090).
`abb-relay`: `ROS_DOMAIN_ID` (0), `RMW_IMPLEMENTATION` (`rmw_cyclonedds_cpp` —
match the host), `ROSBRIDGE_URL`, `FJT_ACTION_NAME`
(`/arm_controller/follow_joint_trajectory`), `SPEED_SCALE` (2.0),
`ACTION_SPEED_SCALE` (1.0), `GOAL_TOLERANCE` (0.02 rad), `GOAL_TIME_SLACK` (8 s),
`STOP_MOTION_SERVICE`, and the `*_TOPIC` overrides.

## Troubleshooting

- **No `/joint_states` on host.** `docker logs abb-relay` should show
  `rosbridge connected; /joint_states relayed xN` with N increasing. If N is
  stuck at 0, `abb_driver` isn't getting robot data — see next point. If N grows
  but the host sees nothing, the relay's `RMW_IMPLEMENTATION` / `ROS_DOMAIN_ID`
  don't match the host (`env | grep -E 'RMW|ROS_DOMAIN'`), or the relay needs
  `--ipc=host` (it has it via compose/run.sh).
- **`abb_driver` not publishing.** `docker logs abb-ros1`. `robot_state` needs
  `controller_joint_names` (set by `robot.launch`) or it segfaults. Check the
  IRC5: `docker exec abb-ros1 ping 192.168.125.1`, ports 11000/11002 open, RAPID
  ROS server running.
- **Robot doesn't move / action aborts with `motion_possible=0`.**
  `ros2 topic echo /robot_status/motion_possible` — if `false`, the controller
  won't execute any motion. Press **Motors On** at the controller/pendant (AUTO
  mode). Also check `/robot_status/e_stopped` and `/robot_status/error_code`.
  A `/stop_motion` call or a protective stop can drop drive power.
- **Action times out (`GOAL_TOLERANCE_VIOLATED`) though the arm moved.** Raise
  `GOAL_TOLERANCE` (rad) or `GOAL_TIME_SLACK` (s) on `abb-relay`.
- **`J23_coupled`.** Leave `false` unless this IRB has the mechanical J2/J3
  parallel linkage.
