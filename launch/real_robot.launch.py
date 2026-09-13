import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Real-hardware counterpart to sim_robot.launch.py.
#
# The real IRB140's arm is NOT driven by anything in this launch file -- it
# comes from a separate bridge stack (not part of this package, started
# independently), which relays the IRC5's ROS 1 abb_driver onto the host
# ROS 2 graph over rosbridge. That bridge already publishes the real
# /joint_states and exposes /arm_controller/follow_joint_trajectory
# directly, so unlike sim_robot.launch.py
# there is no controller_manager here and therefore no joint_state_broadcaster/
# arm_controller/gripper_controller spawners to run -- those only exist to
# talk to gz_ros2_control's in-sim controller_manager.
#
# The gripper has no equivalent in abb_driver (it's a pneumatic on/off signal,
# not a ros2_control joint), so it's driven here by the pneumatic_gripper_controller
# node (src/pneumatic_gripper_controller.cpp) in its real mode (sim:=false, the
# node's own default): it hosts a control_msgs/action/GripperCommand action
# server at gripper_controller/gripper_cmd -- the same action name/type
# abb_irb140_moveit_config's moveit_controllers.yaml already expects, so
# MoveIt's controller manager can drive the real gripper directly, the same
# way it drives the simulated one via gz_ros2_control's GripperActionController.
# On each accepted goal it opens/closes the gripper via the ABB controller's
# Robot Web Services REST API and mocks the 12 finger joints' /joint_states
# (nothing else publishes them on the real robot). robot_state_publisher below
# merges those with the bridge's arm /joint_states into full TF.


def generate_launch_description():
    pkg_share = get_package_share_directory('abb_irb140_description')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz alongside the real robot',
    )
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf.rviz')

    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.125.1',
        description='ABB IRC5 controller IP (Robot Web Services endpoint for the gripper signal)',
    )

    # Same use_octomap/octomap_resolution/octomap_file switch as sim_robot.launch.py:
    #   use_octomap:=true  -> live mapping from an RGBD point cloud on
    #     /camera/depth/color/points, fed by realsense_camera_node below
    #     (requires use_camera:=true, the default).
    #   use_octomap:=false -> static: serve the pre-built octree named by
    #     octomap_file (maps/robot_lab_sim.ot by default), latched, no
    #     per-frame cost. This is the default.
    declare_use_octomap = DeclareLaunchArgument(
        'use_octomap',
        default_value='false',
        description=(
            'true: run octomap_server live-mapping an RGBD point cloud '
            '(requires use_camera:=true so something publishes '
            '/camera/depth/color/points); '
            'false: serve the pre-built octree from octomap_file'
        ),
    )
    declare_octomap_resolution = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.05',
        description='Leaf/voxel size (metres) of the octomap occupancy octree (use_octomap:=true only)',
    )
    declare_octomap_file = DeclareLaunchArgument(
        'octomap_file',
        default_value=os.path.join(pkg_share, 'maps', 'robot_lab_sim.ot'),
        description='Pre-built octree (.ot/.bt) served when use_octomap:=false',
    )

    # Real D405 wrist camera. Configured to match the sim's RealSense-style
    # topic contract (config/rgbd_bridge.yaml / sim_robot.launch.py) exactly:
    # camera_name/camera_namespace avoid the driver's default /camera/camera/...
    # double-nesting, and both optical frame ids are pinned to the URDF's
    # existing depth_camera_optical (urdf/irb140.urdf) -- the single combined
    # RGBD frame the sim also uses. publish_tf is off because
    # robot_state_publisher, driven by the URDF's fixed joints, already
    # publishes the full static chain out to that frame; the driver only
    # needs to stamp its own image/cloud headers with it.
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description=(
            'Start the real D405 RealSense driver, publishing RealSense-style '
            'topics under /camera/...'
        ),
    )

    # Process the xacro at launch time, same as sim_robot.launch.py -- no
    # controllers_yaml mapping needed since there's no controller_manager here
    # to read it; the xacro's own xacro:arg default applies.
    xacro_path = os.path.join(pkg_share, 'urdf', 'irb140.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Real mode: hits the RWS REST API to actuate the pneumatic gripper and
    # mocks the finger joints' /joint_states (see module docstring above).
    pneumatic_gripper_controller_node = Node(
        package='abb_irb140_description',
        executable='pneumatic_gripper_controller',
        name='pneumatic_gripper_controller',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'sim': False,
        }],
    )

    # Subscribes to /tf published by robot_state_publisher_node above, itself
    # driven by the merged real /joint_states (arm from the bridge, gripper
    # fingers mocked by pneumatic_gripper_controller_node).
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        parameters=[{
            'camera_name': 'camera',
            'camera_namespace': '',
            'enable_color': True,
            'enable_depth': True,
            'pointcloud.enable': True,
            'align_depth.enable': True,
            # This driver version has no per-stream frame_id override params
            # (confirmed via `ros2 param list /camera`), so its own optical
            # frames are always camera_color_optical_frame /
            # camera_depth_optical_frame under a camera_link root -- a
            # REP-103 body-convention frame, not directly identity-alignable
            # with the URDF's optical-convention depth_camera_optical. Rather
            # than publish that whole separate chain, publish_tf is off here
            # and the two static_transform_publishers below graft just the
            # leaf optical frames straight onto depth_camera_optical instead.
            'publish_tf': False,
        }],
    )

    # Bridges the URDF's depth_camera_optical (see urdf/irb140.urdf, already
    # in standard ROS optical convention: z forward, x right, y down) onto
    # the RealSense driver's own optical frames with an identity transform --
    # both are optical-convention frames at the same physical camera, and
    # this repo already treats color+depth as one simplified, coincident RGBD
    # frame (matching the sim; see config/rgbd_bridge.yaml). Two separate
    # static transforms (not one, and not to camera_link) because each is a
    # leaf with exactly one parent -- grafting onto camera_link instead would
    # require publish_tf:=true on the driver, duplicating/conflicting with
    # these.
    camera_color_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_optical_to_camera_color_optical_frame',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        arguments=[
            '--frame-id', 'depth_camera_optical',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
    )
    camera_depth_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_optical_to_camera_depth_optical_frame',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        arguments=[
            '--frame-id', 'depth_camera_optical',
            '--child-frame-id', 'camera_depth_optical_frame',
        ],
    )

    # octomap_server subscribes to "cloud_in"; remap it onto the real RGBD
    # driver's point cloud (see declare_use_octomap above). frame_id is the
    # global fixed frame the octree accumulates in (the robot's URDF root is
    # "world"); base_frame_id is used for the ground-plane filter / 2D
    # projection.
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_octomap')),
        parameters=[{
            'resolution': ParameterValue(
                LaunchConfiguration('octomap_resolution'), value_type=float
            ),
            'frame_id': 'world',
            'base_frame_id': 'base_link',
            'sensor_model.max_range': 5.0,
            'filter_ground': False,
            'latch': False,
        }],
        remappings=[('cloud_in', '/camera/depth/color/points')],
    )

    # use_octomap:=false counterpart: same executable, file-load mode -- reads
    # the "octomap_path" parameter and serves that octree latched instead of
    # consuming a live point cloud.
    octomap_static_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_octomap')),
        parameters=[{
            'octomap_path': LaunchConfiguration('octomap_file'),
            'frame_id': 'world',
            'base_frame_id': 'base_link',
            'latch': True,
        }],
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_robot_ip,
        declare_use_octomap,
        declare_octomap_resolution,
        declare_octomap_file,
        declare_use_camera,
        robot_state_publisher_node,
        pneumatic_gripper_controller_node,
        rviz_node,
        realsense_camera_node,
        camera_color_optical_tf_node,
        camera_depth_optical_tf_node,
        octomap_server_node,
        octomap_static_server_node,
    ])
