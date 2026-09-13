#!/usr/bin/env python3
"""Minimal smoke test for the simulated arm: sends one FollowJointTrajectory
goal and waits for the result.

Useful for confirming sim_robot.launch.py's controller stack (controller_manager
+ arm_controller) is up and accepting goals before running the full pick-and-
place sequence in irb140_pick_and_place_demo_real.py. Does not touch the
gripper.
"""
import asyncio

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
TEST_POSITIONS = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
MOVE_DURATION_SEC = 2.0
GOAL_TIME_TOLERANCE_SEC = 10.0


class PickAndPlaceDemoNode(Node):
    def __init__(self):
        super().__init__("pick_and_place_demo_node")
        self.follow_joint_trajectory_cli = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

    async def send_trajectory(self):
        """Send a single trajectory point and wait for the result."""
        self.get_logger().info(f"Moving to test pose {TEST_POSITIONS}...")

        goal = FollowJointTrajectory.Goal(
            trajectory=JointTrajectory(
                joint_names=JOINT_NAMES,
                points=[
                    JointTrajectoryPoint(
                        positions=TEST_POSITIONS,
                        time_from_start=Duration(sec=int(MOVE_DURATION_SEC), nanosec=0),
                    )
                ],
            ),
            goal_time_tolerance=Duration(sec=int(GOAL_TIME_TOLERANCE_SEC), nanosec=0),
        )

        send_goal_future = self.follow_joint_trajectory_cli.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        goal_handle = await self._rclpy_future_to_asyncio(send_goal_future)

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted, waiting for result...")

        result = await self._rclpy_future_to_asyncio(goal_handle.get_result_async())

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal completed successfully.")
        else:
            self.get_logger().error(f"Goal finished with non-success status: {result.status}")

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
    spin_task = asyncio.create_task(asyncio.to_thread(executor.spin))

    # Wait for action server
    if not node.follow_joint_trajectory_cli.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Action server not available!")
        executor.shutdown()
        rclpy.shutdown()
        return

    node.get_logger().info("Action server is available")
    await asyncio.sleep(0.5)

    # Run the demo
    await node.send_trajectory()

    # Cleanup
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
