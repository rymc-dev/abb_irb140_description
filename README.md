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


actuating grippers from remote device of controller, abb driver does not contain implementation for this, instead we utilize RWS on signal closeGrippersOut via this command: 

its importatn to ensure the signal utilzies all opening the gripper 

"""bash
curl --digest -u 'Default User:robotics' \
  -d 'lvalue=1' \
  'http://192.168.125.1/rw/iosystem/signals/closeGrippersOut?action=set'
 """ 

 closing the gripper: 
 """bash

 """
 we just set lvalue=0
