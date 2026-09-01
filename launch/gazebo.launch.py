import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('abb_irb140_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz alongside the simulation',
    )
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf.rviz')

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

    urdf_path = os.path.join(pkg_share, 'urdf', 'irb140.urdf')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    # gz_ros2_control's <parameters> tag isn't resolved through xacro's
    # $(find pkg) here (the URDF is passed through as plain text, not
    # xacro-processed), so substitute the real installed path ourselves.
    robot_description = robot_description.replace(
        'CONTROLLERS_YAML_PATH', controllers_yaml_path
    )

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

    return LaunchDescription([
        declare_use_rviz,
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher_node,
        clock_bridge_node,
        camera_bridge_node,
        spawn_robot_node,
        rviz_node,
        delay_joint_state_broadcaster,
        delay_arm_controller,
    ])
