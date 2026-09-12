import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('abb_irb140_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz alongside the simulation',
    )
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf.rviz')

    # octomap_server publishes /octomap_binary, /octomap_full,
    # /octomap_point_cloud_centers, a 2D /projected_map and the
    # occupied/free marker arrays -- either way, the topic interface is the same:
    #   use_octomap:=true  -> live mapping: consume the wrist RGBD point cloud
    #     (bridged as /camera/depth/color/points in config/rgbd_bridge.yaml) and
    #     build the occupancy octree online (extra CPU cost).
    #   use_octomap:=false -> static: load the pre-built octree named by
    #     octomap_file (maps/robot_lab_sim.ot by default) and serve it latched,
    #     no per-frame cost. This is the default.
    declare_use_octomap = DeclareLaunchArgument(
        'use_octomap',
        default_value='false',
        description=(
            'true: run octomap_server live-mapping the RGBD point cloud; '
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

    # sdformat's urdf->sdf conversion rewrites the URDF's package:// mesh URIs
    # to model://<package_name>/..., which gz-sim resolves by searching
    # GZ_SIM_RESOURCE_PATH for a "<package_name>" subdirectory -- so the
    # share/ directory (one level up from pkg_share) needs to be on that path.
    gz_resource_path_value = os.path.dirname(pkg_share)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path_value += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', gz_resource_path_value
    )

    # Process the xacro at launch time (edits to irb140.xacro take effect on
    # the next launch, no regeneration step). The controllers_yaml mapping
    # feeds gz_ros2_control's <parameters> tag the resolved installed path --
    # xacro evaluates it here, unlike the plain-text URDF path this replaces.
    xacro_path = os.path.join(pkg_share, 'urdf', 'irb140.xacro')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    robot_description = xacro.process_file(
        xacro_path, mappings={'controllers_yaml': controllers_yaml_path}
    ).toxml()

    # robot_lab.world places the robot_lab_podium and robot_lab_table
    # models (the irb140 itself is spawned separately below, on top of
    # the podium, via the ROS-based spawner so gz_ros2_control still gets
    # its resolved CONTROLLERS_YAML_PATH).
    world_path = os.path.join(pkg_share, 'worlds', 'robot_lab.world')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_path} -r'}.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawns the robot into the running gz-sim world by reading URDF/SDF off
    # the /robot_description topic published by robot_state_publisher above.
    # -z matches the robot_lab_podium's top surface height (0.97m, see
    # robot_lab.world) so the robot ends up mounted on top of it.
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_abb_irb140',
        output='screen',
        arguments=[
            '-topic', 'robot_description', '-name', 'abb_irb140',
            '-x', '0.02', '-y', '0.0', '-z', '0.97',
        ],
    )

    # No separate robot_state_publisher / joint_state_publisher_gui here (unlike
    # abb_irb140.launch.py's standalone preview) -- this RViz instance just
    # subscribes to the /tf already published by robot_state_publisher_node
    # above, which is itself driven by the real /joint_states coming from
    # arm_controller/joint_state_broadcaster in the running sim.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': True}],
    )

    # Bridges sim time from gz-sim to ROS so use_sim_time consumers stay in sync.
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    pneumatic_gripper_controller_node = Node(
        package='abb_irb140_description',
        executable='pneumatic_gripper_controller',
        name='pneumatic_gripper_controller',
        output='screen',
        parameters=[{
            'sim': True,
        }],
    )

    # Bridges the rgbd_camera sensor (defined on the depth_camera link in
    # irb140.urdf) from gz-sim into ROS 2 under RealSense-style topic names.
    # The gz<->ROS topic remapping and GZ_TO_ROS direction are declared in
    # config/rgbd_bridge.yaml (passed as the parameter_bridge config_file).
    bridge_config_path = os.path.join(pkg_share, 'config', 'rgbd_bridge.yaml')
    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rgbd_camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
    )

    # config/rgbd_bridge.yaml bridges the raw cloud onto .../points_raw
    # instead of the final .../points name; this node relabels its
    # header.frame_id from the (misleading) "depth_camera_optical" gz-sensors
    # always stamps it with to "depth_camera", which actually matches the
    # data's axes - see scripts/fix_depth_points_frame.py for the full
    # explanation and the gz-sensors upstream issues it's working around.
    fix_depth_points_frame_node = Node(
        package='abb_irb140_description',
        executable='fix_depth_points_frame.py',
        name='fix_depth_points_frame',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # octomap_server subscribes to "cloud_in"; remap it onto the bridged RGBD
    # cloud. frame_id is the global fixed frame the octree accumulates in
    # (the robot's URDF root is "world"); base_frame_id is used for the
    # ground-plane filter / 2D projection. The cloud is published in the
    # "depth_camera" frame (relabelled by fix_depth_points_frame_node above,
    # not by gz-sensors itself), which TF already provides via
    # robot_state_publisher.
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_octomap')),
        parameters=[{
            'use_sim_time': True,
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

    # use_octomap:=false counterpart: the same octomap_server executable, but in
    # file-load mode. OctomapServer's constructor reads the "octomap_path"
    # parameter and calls openFile() at startup, which loads the .ot/.bt octree
    # and publishAll()s it on the latched topics -- identical interface to the
    # live node above. "cloud_in" is left unremapped (nothing publishes it) so
    # the served map stays exactly as built.
    octomap_static_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_octomap')),
        parameters=[{
            'use_sim_time': True,
            'octomap_path': LaunchConfiguration('octomap_file'),
            'frame_id': 'world',
            'base_frame_id': 'base_link',
            'latch': True,
        }],
    )

    rqt_joint_trajectory_controller_node = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_octomap'))
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        output='screen',
        arguments=['arm_controller'],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='gripper_controller_spawner',
        output='screen',
        arguments=['gripper_controller'],
    )

    # gz_ros2_control's plugin (loaded via the URDF's <gazebo> tag) brings up
    # the controller_manager inside gz-sim once the robot entity exists, so
    # controller spawning is chained off the spawn process finishing rather
    # than started unconditionally at launch time.
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_use_octomap,
        declare_octomap_resolution,
        declare_octomap_file,
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher_node,
        clock_bridge_node,
        camera_bridge_node,
        fix_depth_points_frame_node,
        spawn_robot_node,
        rviz_node,
        octomap_server_node,
        octomap_static_server_node,
        rqt_joint_trajectory_controller_node,
        delay_joint_state_broadcaster,
        delay_arm_controller,
        delay_gripper_controller,
        pneumatic_gripper_controller_node,
    ])
