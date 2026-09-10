#!/usr/bin/env python3
"""
Relay between a ROS 1 rosbridge WebSocket and the native ROS 2 (Jazzy) graph.

ROS 1 (abb_driver, noetic container) --rosbridge ws--> this node --> ROS 2, and
ROS 2 --> this node --> ROS 1 for commands / the FollowJointTrajectory action.

Provides on the ROS 2 side:
  /joint_states                              sensor_msgs/msg/JointState        (from robot)
  /feedback_states                           control_msgs/.../FollowJointTrajectoryFeedback
  /robot_status/json                         std_msgs/msg/String  (RobotStatus as JSON)
  /robot_status/{in_motion,e_stopped}        std_msgs/msg/Bool
  /robot_status/error_code                   std_msgs/msg/Int32
  /joint_path_command                        trajectory_msgs/msg/JointTrajectory  (to robot)
  <FJT_ACTION_NAME>                          control_msgs/action/FollowJointTrajectory
  <STOP_MOTION_SERVICE>                      std_srvs/srv/Trigger

ROS 1 Header is secs/nsecs/seq; ROS 2 is sec/nanosec and has no seq.

Env vars:
  ROSBRIDGE_URL              ws://127.0.0.1:9090
  JOINT_STATES_TOPIC         /joint_states
  ROBOT_STATUS_TOPIC         /robot_status
  JOINT_PATH_COMMAND_TOPIC   /joint_path_command
  FEEDBACK_STATES_TOPIC      /feedback_states
  FJT_ACTION_NAME            /arm_controller/follow_joint_trajectory  (what MoveIt calls)
  STOP_MOTION_SERVICE        /stop_motion
  SPEED_SCALE               1.0  -- scales the /joint_path_command TOPIC path only
  ACTION_SPEED_SCALE        1.0  -- scales trajectories sent through the ACTION (leave 1.0
                                   for MoveIt; use MoveIt's own velocity scaling instead)
  MIN_POINT_DT             0.05  -- floor on a scaled inter-point interval (s)
  GOAL_TOLERANCE           0.02  -- rad/joint; action succeeds within this of the last point
  GOAL_TIME_SLACK           8.0  -- s allowed past the trajectory end before abort

The FJT action publishes the trajectory straight to /joint_path_command and
judges completion from /joint_states + /robot_status (it does not proxy the ROS 1
/joint_trajectory_action, whose leading empty-path STOP wedges the ABB driver).
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time
from std_msgs.msg import Header, Bool, Int32, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

# ROS 2 has no standalone FollowJointTrajectoryFeedback message; the action's
# Feedback type carries the identical fields (header/joint_names/desired/actual/error).
FJTFeedback = FollowJointTrajectory.Feedback

import roslibpy


# --------------------------------------------------------------------------- #
#  small conversion helpers
# --------------------------------------------------------------------------- #
def r1_time_to_r2(d):
    if not d:
        return Time()
    return Time(sec=int(d.get("secs", d.get("sec", 0))),
                nanosec=int(d.get("nsecs", d.get("nanosec", 0))))


def r1_header_to_r2(d):
    d = d or {}
    return Header(stamp=r1_time_to_r2(d.get("stamp")),
                 frame_id=str(d.get("frame_id", "")))


def secs_to_r1(t):
    return {"secs": int(t), "nsecs": int(round((t % 1.0) * 1e9))}


def r2_dur_secs(d):
    return d.sec + d.nanosec * 1e-9


def r1_point_to_r2(d):
    p = JointTrajectoryPoint()
    p.positions = [float(x) for x in d.get("positions", [])]
    p.velocities = [float(x) for x in d.get("velocities", [])]
    p.accelerations = [float(x) for x in d.get("accelerations", [])]
    p.effort = [float(x) for x in d.get("effort", [])]
    tfs = d.get("time_from_start", {})
    p.time_from_start.sec = int(tfs.get("secs", tfs.get("sec", 0)))
    p.time_from_start.nanosec = int(tfs.get("nsecs", tfs.get("nanosec", 0)))
    return p


def r2_traj_to_r1(traj: JointTrajectory, scale: float, min_dt: float):
    pts = []
    for i, p in enumerate(traj.points):
        t = r2_dur_secs(p.time_from_start)
        if scale != 1.0 and t > 0.0:
            t = max(t / scale, (i + 1) * min_dt)
        pts.append({
            "positions": list(p.positions),
            "velocities": [v * scale for v in p.velocities],
            "accelerations": [a * scale * scale for a in p.accelerations],
            "effort": list(p.effort),
            "time_from_start": secs_to_r1(t),
        })
    return {
        "header": {"seq": 0,
                   "stamp": {"secs": traj.header.stamp.sec,
                             "nsecs": traj.header.stamp.nanosec},
                   "frame_id": traj.header.frame_id},
        "joint_names": list(traj.joint_names),
        "points": pts,
    }


# --------------------------------------------------------------------------- #
class RelayNode(Node):
    def __init__(self):
        super().__init__("abb_ros1_relay")
        env = os.environ.get
        self._js_count = 0            # set before any rosbridge callback can fire
        self._last_js = {}            # joint_name -> latest position (rad)
        self._in_motion = False       # from /robot_status
        self._motion_possible = False
        self._got_robot_status = False

        url = env("ROSBRIDGE_URL", "ws://127.0.0.1:9090")
        self.t_js = env("JOINT_STATES_TOPIC", "/joint_states")
        self.t_rs = env("ROBOT_STATUS_TOPIC", "/robot_status")
        self.t_jpc = env("JOINT_PATH_COMMAND_TOPIC", "/joint_path_command")
        self.t_fb = env("FEEDBACK_STATES_TOPIC", "/feedback_states")
        self.fjt_action_name = env("FJT_ACTION_NAME",
                                   "/arm_controller/follow_joint_trajectory")
        self.ros1_fjt_action = env("ROS1_FJT_ACTION", "/joint_trajectory_action")
        self.stop_srv_name = env("STOP_MOTION_SERVICE", "/stop_motion")
        self.speed_scale = float(env("SPEED_SCALE", "1.0"))
        self.action_speed_scale = float(env("ACTION_SPEED_SCALE", "1.0"))
        self.min_point_dt = float(env("MIN_POINT_DT", "0.05"))
        self.goal_tol = float(env("GOAL_TOLERANCE", "0.02"))       # rad, per joint
        self.goal_time_slack = float(env("GOAL_TIME_SLACK", "8.0"))  # s past traj end

        host, port = url.replace("ws://", "").split(":")
        self.get_logger().info(f"connecting to rosbridge at {url}")
        self._rbc = roslibpy.Ros(host=host, port=int(port))
        self._rbc.on_ready(lambda: self.get_logger().info("rosbridge connected"))
        self._rbc.run()

        cbg = ReentrantCallbackGroup()
        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)

        # ---- ROS 2 publishers (robot -> ROS 2) ----
        self.pub_js = self.create_publisher(JointState, self.t_js, sensor_qos)
        self.pub_fb = self.create_publisher(FJTFeedback, self.t_fb, 10)
        self.pub_rs_json = self.create_publisher(String, self.t_rs + "/json", 10)
        self.pub_rs_motion = self.create_publisher(Bool, self.t_rs + "/in_motion", 10)
        self.pub_rs_estop = self.create_publisher(Bool, self.t_rs + "/e_stopped", 10)
        self.pub_rs_err = self.create_publisher(Int32, self.t_rs + "/error_code", 10)
        self.pub_rs_mposs = self.create_publisher(Bool, self.t_rs + "/motion_possible", 10)
        self.pub_rs_drives = self.create_publisher(Bool, self.t_rs + "/drives_powered", 10)

        # ---- rosbridge subscriptions (ROS 1 -> ROS 2) ----
        roslibpy.Topic(self._rbc, self.t_js, "sensor_msgs/JointState",
                       queue_length=1, throttle_rate=0).subscribe(self._on_joint_states)
        roslibpy.Topic(self._rbc, self.t_rs, "industrial_msgs/RobotStatus",
                       queue_length=1, throttle_rate=0).subscribe(self._on_robot_status)
        roslibpy.Topic(self._rbc, self.t_fb, "control_msgs/FollowJointTrajectoryFeedback",
                       queue_length=1, throttle_rate=0).subscribe(self._on_feedback_states)

        # ---- /joint_path_command : ROS 2 -> ROS 1 ----
        self._rb_jpc = roslibpy.Topic(self._rbc, self.t_jpc,
                                      "trajectory_msgs/JointTrajectory")
        self._rb_jpc.advertise()
        self.create_subscription(JointTrajectory, self.t_jpc,
                                 self._on_joint_path_command, 10,
                                 callback_group=cbg)

        # ---- FollowJointTrajectory action ----
        # We do NOT proxy to the ROS 1 /joint_trajectory_action: that node
        # prefixes an empty (STOP) /joint_path_command that wedges the ABB
        # download interface. Instead publish the trajectory straight to
        # /joint_path_command (the path that actually moves the robot) and
        # judge completion here from /joint_states + /robot_status.
        self._fjt_server = ActionServer(
            self, FollowJointTrajectory, self.fjt_action_name,
            execute_callback=self._execute_fjt,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cbg)
        self.get_logger().info(
            f"FollowJointTrajectory action server: {self.fjt_action_name} "
            f"(drives /joint_path_command directly)")

        # ---- /stop_motion : ROS 2 service -> ROS 1 service ----
        self._rb_stop = roslibpy.Service(self._rbc, self.stop_srv_name,
                                         "industrial_msgs/StopMotion")
        self.create_service(Trigger, self.stop_srv_name, self._on_stop_motion,
                            callback_group=cbg)
        self.get_logger().info(f"stop service: {self.stop_srv_name} (std_srvs/Trigger)")

        self.create_timer(5.0, self._report)

    # ----------------- ROS 1 -> ROS 2 topic relays -----------------
    def _on_joint_states(self, m):
        js = JointState()
        js.header = r1_header_to_r2(m.get("header"))
        js.name = list(m.get("name", []))
        js.position = [float(x) for x in m.get("position", [])]
        js.velocity = [float(x) for x in m.get("velocity", [])]
        js.effort = [float(x) for x in m.get("effort", [])]
        self.pub_js.publish(js)
        self._js_count += 1
        if js.name and js.position:
            self._last_js = dict(zip(js.name, js.position))

    def _on_feedback_states(self, m):
        fb = FJTFeedback()
        fb.header = r1_header_to_r2(m.get("header"))
        fb.joint_names = list(m.get("joint_names", []))
        fb.desired = r1_point_to_r2(m.get("desired", {}))
        fb.actual = r1_point_to_r2(m.get("actual", {}))
        fb.error = r1_point_to_r2(m.get("error", {}))
        self.pub_fb.publish(fb)

    def _on_robot_status(self, m):
        self.pub_rs_json.publish(String(data=json.dumps(m)))

        def tri(x):
            return bool(isinstance(x, dict) and x.get("val", -1) == 1)
        self._in_motion = tri(m.get("in_motion", {}))
        self._motion_possible = tri(m.get("motion_possible", {}))
        self._got_robot_status = True
        self.pub_rs_motion.publish(Bool(data=self._in_motion))
        self.pub_rs_estop.publish(Bool(data=tri(m.get("e_stopped", {}))))
        self.pub_rs_err.publish(Int32(data=int(m.get("error_code", 0))))
        self.pub_rs_mposs.publish(Bool(data=self._motion_possible))
        self.pub_rs_drives.publish(Bool(data=tri(m.get("drives_powered", {}))))

    # ----------------- ROS 2 -> ROS 1 : /joint_path_command -----------------
    def _on_joint_path_command(self, msg: JointTrajectory):
        out = r2_traj_to_r1(msg, self.speed_scale, self.min_point_dt)
        self._rb_jpc.publish(roslibpy.Message(out))
        self.get_logger().info(
            f"forwarded JointTrajectory ({len(out['points'])} pts, "
            f"speed_scale={self.speed_scale}) to ROS 1")

    # ----------------- FollowJointTrajectory action -----------------
    def _stop_robot(self):
        try:  # empty path = STOP for the ABB download interface
            self._rb_jpc.publish(roslibpy.Message(
                {"header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
                 "joint_names": [], "points": []}))
        except Exception:  # noqa: BLE001
            pass
        try:
            self._rb_stop.call(roslibpy.ServiceRequest({}), timeout=3)
        except Exception:  # noqa: BLE001
            pass

    def _mk_result(self, code, msg=""):
        r = FollowJointTrajectory.Result()
        r.error_code = int(code)
        r.error_string = str(msg)
        return r

    def _execute_fjt(self, goal_handle):
        traj = goal_handle.request.trajectory
        jn = list(traj.joint_names)
        if not traj.points or not jn:
            goal_handle.abort()
            return self._mk_result(FollowJointTrajectory.Result.INVALID_GOAL,
                                   "empty trajectory")
        self.get_logger().info(
            f"FJT goal: {len(traj.points)} pts, joints={jn}, "
            f"action_speed_scale={self.action_speed_scale}")

        if self._got_robot_status and not self._motion_possible:
            msg = ("robot reports motion_possible=0 (drives/motors off or "
                   "protective stop) - enable Motors On at the controller/pendant")
            self.get_logger().warn("FJT goal REJECTED: " + msg)
            goal_handle.abort()
            return self._mk_result(FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                   msg)

        # send straight to the ABB download interface (no leading STOP)
        out = r2_traj_to_r1(traj, self.action_speed_scale, self.min_point_dt)
        self._rb_jpc.publish(roslibpy.Message(out))

        last = traj.points[-1]
        target = dict(zip(jn, last.positions))
        traj_dur = r2_dur_secs(last.time_from_start)
        if self.action_speed_scale != 1.0 and traj_dur > 0.0:
            traj_dur /= self.action_speed_scale
        t0 = time.monotonic()
        deadline = t0 + traj_dur + self.goal_time_slack
        started_moving = False

        while True:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("FJT goal CANCELED -> stopping robot")
                self._stop_robot()
                goal_handle.canceled()
                return self._mk_result(0, "canceled")

            cur = dict(self._last_js)
            elapsed = time.monotonic() - t0
            errs = [abs(target[j] - cur[j]) for j in jn if j in cur]
            max_err = max(errs) if errs else float("inf")

            # feedback
            try:
                fb = FJTFeedback()
                fb.header.stamp = self.get_clock().now().to_msg()
                fb.joint_names = jn
                fb.desired = last
                act = JointTrajectoryPoint()
                act.positions = [cur.get(j, math.nan) for j in jn]
                fb.actual = act
                ep = JointTrajectoryPoint()
                ep.positions = [target[j] - cur[j] if j in cur else math.nan for j in jn]
                fb.error = ep
                goal_handle.publish_feedback(fb)
            except Exception:  # noqa: BLE001
                pass

            if self._in_motion:
                started_moving = True

            settled = (not self._in_motion) and (started_moving or elapsed >= min(1.5, traj_dur or 1.5))
            if max_err <= self.goal_tol and settled:
                goal_handle.succeed()
                self.get_logger().info(f"FJT goal SUCCEEDED (max_err={max_err:.4f} rad)")
                return self._mk_result(0, "")

            if time.monotonic() > deadline:
                goal_handle.abort()
                m = f"timeout after {elapsed:.1f}s, max_err={max_err:.4f} rad (in_motion={self._in_motion})"
                self.get_logger().warn("FJT goal ABORTED: " + m)
                return self._mk_result(
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, m)

            time.sleep(0.05)

    # ----------------- ROS 2 -> ROS 1 : /stop_motion -----------------
    def _on_stop_motion(self, request, response):
        try:
            r = self._rb_stop.call(roslibpy.ServiceRequest({}), timeout=5)
            code = (r or {}).get("code", {})
            val = code.get("val") if isinstance(code, dict) else None
            response.success = (val == 1)  # ServiceReturnCode.SUCCESS
            response.message = f"industrial_msgs/StopMotion code.val={val}"
        except Exception as e:  # noqa: BLE001
            response.success = False
            response.message = f"stop_motion call failed: {e}"
        self.get_logger().info(f"/stop_motion -> {response.success} ({response.message})")
        return response

    # ----------------------------------------------------------------
    def _report(self):
        state = "connected" if self._rbc.is_connected else "DISCONNECTED"
        self.get_logger().info(
            f"rosbridge {state}; /joint_states relayed x{self._js_count}")

    def destroy_node(self):
        try:
            self._rbc.terminate()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RelayNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
