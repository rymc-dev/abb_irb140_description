import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from enum import Enum, auto
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from builtin_interfaces.msg import Duration

from std_srvs.srv import Trigger

from sensor_msgs.msg import JointState

class GripperState(Enum):
    OPEN = auto()
    CLOSED = auto()

class TransitionState(Enum):
    OPENING = auto()
    CLOSING = auto()

class GripperControllerNode(Node):

    def __init__(self):
        super().__init__("controller", namespace="gripper")
        
        self.transition_state = None
        self.gripper_state = None
        self.current_joint_state: JointState = None

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            qos_profile=qos_profile_system_default,
            callback_group=ReentrantCallbackGroup()
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "joint_states",
            callback=joint_state_cb,
            qos_profile=qos_profile_system_default,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.toggle_gripper_srv = self.create_service(
            Trigger,
            "toggle_gripper",
            ReentrantCallbackGroup()
            callback=lambda msg: 
        )
        
        self.gripper_toggle_tmr = self.create_timer(
            timer_period_sec = 0.001,
            callback=self.pub_joint_state_update,
            callback_group=ReentrantCallbackGroup(),
            clock=self.get_clock(),
            autostart=False
        )

    def joint_state_cb(self, msg: JointState): 
        self.current_joint_state = msg

    def toggle_gripper_cb(self, msg: Trigger):
        try:
            if self.gripper_state is GripperState.OPEN:
                self.transition_state = TransitionState.CLOSING
            elif self.gripper_state is GripperState.CLOSED:
                self.transition_state = TransitionState.OPENING
                
            self.gripper_toggle_tmr.reset()
            
            self.get_clock().sleep_for(self.get_clock().now() + Duration(sec=1))
            
            self.gripper_toggle_tmr.cancel()

            rsp = Trigger.Response(
                success = True,
                message = "Grippers successfully Triggered"
            )

            if self.gripper_state is GripperState.OPEN:
                self.gripper_state = GripperState.CLOSED
            elif self.gripper_state is GripperState.CLOSED:
                self.gripper_state = GripperState.OPEN
            
        
        except Exception as e: 
            rsp = Trigger.Response(
                success = False,
                message = f"Grippers unsuccessfully Triggered due to: '{str(e)}'"
            )        
        
        self.transition_state = None 

        return rsp 

    def pub_joint_state_update(self):
        # upper = 0.19, # lower = 0.0, 
        # for both right and left gripper,
        # we wish to toggle gripper in 1 second therefore
        # we simple update gripper by 0.0019 if closing state
        # else -0.0019 if opening
        if self.transition_state == TransitionState.OPENING:
            # next_joint_state: JointState = self.current_joint_state 
            # next_joint_state.position.index 
            self.joint_state_pub.publish(JointState()) 
            
        elif self.transition_state ==  TransitionState.CLOSING: 
            self.joint_state_pub.publish(JointState())

        else: 
            self.get_logger().error("something has gone critically wrong publishing joint states for opening/closing pneumatic grippers.")            


def main():
    rclpy.init()

    node_1 = GripperControllerNode()
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node_1)
        executor.spin()

    except Exception as e:
        print (str(e))
        
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()