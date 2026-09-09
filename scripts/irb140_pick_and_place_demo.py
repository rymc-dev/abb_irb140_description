import rclpy
from rclpy.node import Node
import asyncio
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

class PickAndPlaceDemoNode(Node):
    def __init__(self): 
        super().__init__(
            "pick_and_place_demo_node",
            namespace="abb_irb140"
        )
        self.follow_joint_trajectory_cli = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )
        self.goal_done_event = asyncio.Event()
        self.goal_result = None

    async def send_trajectory(self):
        """Send a single trajectory point"""
        self.get_logger().info("Moving to Joint State 1...")
        
        goal = FollowJointTrajectory.Goal(
            trajectory=JointTrajectory(
                joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
                points=[
                    JointTrajectoryPoint(
                        positions=[0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        time_from_start=Duration(sec=2, nanosec=0)
                    )
                ]
            ),
            goal_time_tolerance=Duration(sec=10, nanosec=0)
        )
        
        # Send goal and wait for response
        send_goal_future = self.follow_joint_trajectory_cli.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        # Convert rclpy Future to asyncio-compatible coroutine
        goal_handle = await self._rclpy_future_to_asyncio(send_goal_future)
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return
        
        self.get_logger().info("Goal accepted!")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result = await self._rclpy_future_to_asyncio(result_future)
        
        self.get_logger().info(f"Goal completed with status: {result.status}")

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
        # Inspect what's actually in the feedback
        self.get_logger().info(f"Feedback received: {feedback}")

async def main():
    rclpy.init()
    node = PickAndPlaceDemoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Spin executor in background
    spin_task = asyncio.create_task(
        asyncio.to_thread(executor.spin)
    )
    
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