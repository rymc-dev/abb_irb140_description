# abb_irb140_description

A ROS2 Humble description package for the ABB_IRB140 robotic platform. 


initial joint positions: 
```bash
$ ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      { positions: [0.0, 0.7, -0.5, 0.0, 1.4, 0.0],
        time_from_start: { sec: 2, nanosec: 0 } }
    ]
  }
}"
```