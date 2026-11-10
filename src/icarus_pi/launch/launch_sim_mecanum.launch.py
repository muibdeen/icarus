import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_sim_mecanum(context):
    package_name = 'icarus_pi'
    world_name = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    gui = LaunchConfiguration('gui').perform(context)

    # Resolve world path
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_name)

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Gz Sim (Jazzy)
    # Use ros_gz_sim package's gz_server or gz_sim launch.
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gz_args = f"-v 4 -r {world_path}"
    if gui.lower() != 'true':
        gz_args += " -s"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'false'
        }.items()
    )

    # ROS <-> Gz bridge for sim clock + TF (required for controller_manager use_sim_time)
    # Gz publishes /clock on gz transport; ROS nodes expect rosgraph_msgs/Clock on /clock.
 #   echo "Bridging clock GZ_TO_ROS only (avoid duplicate publisher conflict)"


    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock[gz',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V[gz',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V[gz',
        ],
        output='screen',
    )

    # In Gz Sim, ros2_control runs inside gz server via gz_ros2_control plugin (loaded from URDF).
    # Still need controller_manager spawners for the controllers defined in my_controllers.yaml
    # if not already auto-loaded. Gz plugin loads controller_manager params file.
    controller_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mech_cont", "--controller-manager", "/controller_manager"],
    )


    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # Spawn robot entity via ros_gz_sim create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'icarus_pi',
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.07'
        ],
        output='screen'
    )

    # Give gz sim time to start before spawning
    from launch.actions import TimerAction
    delayed_spawn = TimerAction(period=2.0, actions=[spawn_entity])
    delayed_spawner = TimerAction(period=4.0, actions=[controller_drive_spawner, joint_broad_spawner])

    return [
        rsp,
        joint_state_publisher_node,
        gz_sim,
        clock_bridge,
        delayed_spawn,
        joystick,
        delayed_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.world', description='World file name (*.world) in icarus_pi/worlds'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_sim_mecanum)
    ])
