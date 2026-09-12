#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Real joint-space waypoints, captured from the live robot via
# `ros2 topic echo /joint_states --once` while jogging it by hand.
# Tune these before running against a different setup.
WAYPOINTS = {
    "pre_pick": [-0.12438856065273285, 0.5640290379524231, -0.4248206317424774,
                 0.03683801367878914, 1.333264946937561, -0.16127046942710876],
    "pick": [-0.12439039349555969, 0.9279851317405701, -0.18804606795310974,
             0.05352947860956192, 0.7330714464187622, -0.1924125701189041],
    "post_pick": [-0.12439223378896713, 0.5797401666641235, -0.3686411678791046,
                  0.037661582231521606, 1.2614178657531738, -0.16406504809856415],
    "move_to_box": [
        -0.38488104939460754, 1.0068862438201904, -1.0835121870040894, 
        0.06004104018211365, 1.5615001916885376, -0.4128890931606293
    ],
    "return_to_base": [
        -0.12438856065273285, 0.5640290379524231, -0.4248206317424774,
        0.03683801367878914, 1.333264946937561, -0.16127046942710876
    ]
}

# Move durations used to be a flat 20s per waypoint regardless of how far
# that waypoint actually was -- e.g. pre_pick->pick is a ~0.6 rad move on the
# biggest joint, so 20s worked out to ~0.03 rad/s (~1.7 deg/s). Instead, scale
# each move's duration to its largest single-joint delta so nearby waypoints
# aren't held to the same time budget as far ones. Tune the pace here (safe to
# push faster: the docker/abb-ros1-bridge relay's GOAL_TIME_SLACK gives plenty
# of deadline margin past this, and move_to() advances the instant
# /joint_states shows we're actually within tolerance regardless of whether
# the robot hits this exact pace):
NOMINAL_JOINT_SPEED_RAD_S = 0.4  # per-joint angular speed budget
MIN_MOVE_DURATION_SEC = 1.5      # floor so small moves aren't snappy/jerky
# pre_pick's start pose isn't known ahead of time (wherever the arm happens to
# be when the demo starts), so its budget is sized off an assumed worst-case
# delta instead of a computed one.
FIRST_MOVE_MAX_EXPECTED_DELTA_RAD = 1.5
FIRST_MOVE_DURATION_SEC = max(MIN_MOVE_DURATION_SEC,
                               FIRST_MOVE_MAX_EXPECTED_DELTA_RAD / NOMINAL_JOINT_SPEED_RAD_S)

# How close (rad, per joint) counts as "arrived" -- matches the ABB bridge's
# own GOAL_TOLERANCE default (docker/abb-ros1-bridge/relay/relay_node.py) so
# both sides agree on what "reached" means.
JOINT_TOLERANCE_RAD = 0.02

GOAL_TIME_TOLERANCE_SEC = 10.0


def move_duration_sec(start_positions: list, target_positions: list) -> float:
    """Scale a move's duration to its largest single-joint delta."""
    max_delta = max(abs(t - s) for s, t in zip(start_positions, target_positions))
    return max(MIN_MOVE_DURATION_SEC, max_delta / NOMINAL_JOINT_SPEED_RAD_S)


class PickAndPlaceDemoNode(Node):
    def __init__(self):
        super().__init__("pick_and_place_demo_node")
        self.follow_joint_trajectory_cli = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_trigger_cli = self.create_client(SetBool, "/gripper_trigger")

        # Latest /joint_states, name -> position, used to detect "arrived
        # within tolerance" ourselves rather than only trusting the
        # controller/bridge's own goal-tolerance bookkeeping.
        self._joint_positions = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        # Goals we advanced past early (see move_to) whose result is still
        # resolving in the background. Tracked so main() can drain them
        # before shutdown instead of leaving them to be torn down mid-flight
        # (asyncio.run() cancelling a still-pending task on exit surfaces as
        # an unhandled CancelledError in its done-callback).
        self._background_tasks: set = set()

    def _on_joint_states(self, msg: JointState):
        self._joint_positions.update(zip(msg.name, msg.position))

    def _within_tolerance(self, target_positions: list, tolerance: float) -> bool:
        if not all(j in self._joint_positions for j in JOINT_NAMES):
            return False
        return all(
            abs(self._joint_positions[j] - t) <= tolerance
            for j, t in zip(JOINT_NAMES, target_positions)
        )

    async def _wait_until_reached(self, target_positions: list, tolerance: float,
                                   poll_period_sec: float = 0.05, settle_checks: int = 2):
        """Poll /joint_states until target_positions are within tolerance for
        settle_checks consecutive polls (a cheap debounce against a single
        noisy/stale sample)."""
        consecutive = 0
        while consecutive < settle_checks:
            if self._within_tolerance(target_positions, tolerance):
                consecutive += 1
            else:
                consecutive = 0
            await asyncio.sleep(poll_period_sec)

    async def move_to(self, name: str, positions: list, duration_sec: float,
                       tolerance: float = JOINT_TOLERANCE_RAD) -> bool:
        """Send a single-point FollowJointTrajectory goal, then advance as
        soon as /joint_states shows we're within tolerance of the target --
        whichever comes first between that and the controller/bridge's own
        result. This way we're not stuck waiting out the full commanded
        duration once we've actually arrived, and we're not solely trusting
        a controller whose own goal-tolerance checking might be disabled.

        Returns True if we can consider the move done (arrived within
        tolerance, or the goal succeeded), False otherwise.
        """
        self.get_logger().info(f"Moving to '{name}' ({duration_sec:.1f}s)...")

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)

        goal = FollowJointTrajectory.Goal(
            trajectory=JointTrajectory(
                joint_names=JOINT_NAMES,
                points=[
                    JointTrajectoryPoint(
                        positions=positions,
                        time_from_start=Duration(sec=sec, nanosec=nanosec)
                    )
                ]
            ),
            goal_time_tolerance=Duration(sec=int(GOAL_TIME_TOLERANCE_SEC), nanosec=0)
        )

        send_goal_future = self.follow_joint_trajectory_cli.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        goal_handle = await self._rclpy_future_to_asyncio(send_goal_future)

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal '{name}' rejected!")
            return False

        self.get_logger().info(f"Goal '{name}' accepted, waiting for result...")

        result_task = asyncio.ensure_future(
            self._rclpy_future_to_asyncio(goal_handle.get_result_async())
        )
        reached_task = asyncio.ensure_future(
            self._wait_until_reached(positions, tolerance)
        )

        done, _pending = await asyncio.wait(
            {result_task, reached_task}, return_when=asyncio.FIRST_COMPLETED
        )

        if reached_task in done:
            self.get_logger().info(f"Reached '{name}' (within {tolerance} rad).")
            # Don't cancel the still-running goal here: on the ABB bridge,
            # canceling calls _stop_robot(), which publishes an empty/STOP
            # trajectory to /joint_path_command -- and cancel_goal_async()
            # only waits for the cancel *request* to be acked, not for the
            # bridge's execute loop to actually act on it (it polls every
            # 50ms). That STOP can then land after we've already sent the
            # *next* goal's trajectory, clobbering it before the arm moves
            # (this is exactly what caused the following goal to time out
            # and abort). Sending the next goal's trajectory is itself
            # enough to supersede this one on /joint_path_command, so just
            # let this goal's result resolve on its own in the background --
            # tracked so main() can drain it before shutdown.
            self._background_tasks.add(result_task)
            result_task.add_done_callback(self._background_tasks.discard)
            result_task.add_done_callback(
                lambda f, name=name: self._log_late_result(name, f)
            )
            return True

        # The goal finished (succeeded/aborted/rejected/canceled) before we
        # ever saw joint_states settle within tolerance.
        reached_task.cancel()
        result = result_task.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Goal '{name}' finished with non-success status: {result.status}"
            )
            return False

        self.get_logger().info(f"Reached '{name}'.")
        return True

    def _log_late_result(self, name: str, result_task: "asyncio.Task"):
        """Log how a goal we'd already advanced past (via tolerance) eventually
        resolved. We superseded it by sending the next goal's trajectory
        rather than canceling it (see move_to), so it may well time out on
        its own stale target once the arm has moved on -- that's expected,
        not an error, so this is purely informational."""
        try:
            result = result_task.result()
        except asyncio.CancelledError:
            # Expected at shutdown: main() cancels any still-pending
            # background tasks itself before tearing down rclpy.
            self.get_logger().debug(f"Goal '{name}' result task canceled (shutdown).")
            return
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"Goal '{name}' result future errored after being superseded: {e}")
            return
        self.get_logger().debug(
            f"Goal '{name}' (already superseded) later resolved with status {result.status}."
        )

    async def set_gripper(self, closed: bool) -> bool:
        """Call /gripper_trigger to open (False) or close (True) the gripper."""
        action = "Closing" if closed else "Opening"
        self.get_logger().info(f"{action} gripper...")

        request = SetBool.Request(data=closed)
        response_future = self.gripper_trigger_cli.call_async(request)
        response = await self._rclpy_future_to_asyncio(response_future)

        if not response.success:
            self.get_logger().error(f"Gripper trigger failed: {response.message}")
            return False

        self.get_logger().info(f"Gripper trigger succeeded: {response.message}")
        return True

    async def run_pick_sequence(self) -> bool:
        """Open gripper, approach, pick, close gripper, retreat, move to box,
        release, then return to base."""
        if not await self.set_gripper(closed=False):
            return False

        if not await self.move_to("pre_pick", WAYPOINTS["pre_pick"],
                                   FIRST_MOVE_DURATION_SEC):
            return False

        if not await self.move_to("pick", WAYPOINTS["pick"],
                                   move_duration_sec(WAYPOINTS["pre_pick"], WAYPOINTS["pick"])):
            return False

        if not await self.set_gripper(closed=True):
            return False

        if not await self.move_to("post_pick", WAYPOINTS["post_pick"],
                                   move_duration_sec(WAYPOINTS["pick"], WAYPOINTS["post_pick"])):
            return False

        if not await self.move_to("move_to_box", WAYPOINTS["move_to_box"],
                                   move_duration_sec(WAYPOINTS["post_pick"], WAYPOINTS["move_to_box"])):
            return False

        if not await self.set_gripper(closed=False):
            return False

        if not await self.move_to("return_to_base", WAYPOINTS["return_to_base"],
                                   move_duration_sec(WAYPOINTS["move_to_box"], WAYPOINTS["return_to_base"])):
            return False

        return True

    async def _rclpy_future_to_asyncio(self, rclpy_future):
        """Convert rclpy Future to asyncio-compatible coroutine"""
        loop = asyncio.get_event_loop()
        asyncio_future = loop.create_future()

        def done_callback(f):
            try:
                result = f.result()
                loop.call_soon_threadsafe(asyncio_future.set_result, result)
            except Exception as e:
                loop.call_soon_threadsafe(asyncio_future.set_exception, e)

        rclpy_future.add_done_callback(done_callback)
        return await asyncio_future

    def feedback_callback(self, feedback):
        """Handle feedback from the action server"""
        self.get_logger().debug(f"Feedback received: {feedback}")


async def main():
    rclpy.init()
    node = PickAndPlaceDemoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin executor in background
    spin_task = asyncio.create_task(
        asyncio.to_thread(executor.spin)
    )

    # Wait for the arm action server and the gripper service
    if not node.follow_joint_trajectory_cli.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Action server not available!")
        executor.shutdown()
        rclpy.shutdown()
        return

    node.get_logger().info("Action server is available")

    if not node.gripper_trigger_cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("Gripper trigger service not available!")
        executor.shutdown()
        rclpy.shutdown()
        return

    node.get_logger().info("Gripper trigger service is available")
    await asyncio.sleep(0.5)

    # Run the pick sequence
    success = await node.run_pick_sequence()
    if success:
        node.get_logger().info("Pick sequence completed successfully.")
    else:
        node.get_logger().error("Pick sequence failed — see errors above.")

    # Drain any goals we advanced past early (see move_to) and left resolving
    # in the background, so nothing gets torn down mid-flight by asyncio.run()
    # cancelling leftover tasks once main() returns (which surfaces as an
    # unhandled CancelledError in their done-callbacks).
    if node._background_tasks:
        for t in list(node._background_tasks):
            t.cancel()
        await asyncio.gather(*node._background_tasks, return_exceptions=True)

    # Cleanup
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
