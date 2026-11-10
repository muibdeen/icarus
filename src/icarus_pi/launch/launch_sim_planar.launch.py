import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_sim_planar(context):
    package_name = 'icarus_pi'
    world_name = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    gui = LaunchConfiguration('gui').perform(context)

    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_name)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'false'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Gz Sim planar controller uses gz plugin planar_controller (from gz_ros2_control via GzSimSystem)
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gz_args = f"-v 4 -r {world_path}"
    if gui.lower() != 'true':
        gz_args += " -s"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'false'}.items()
    )

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

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'icarus_pi', '-allow_renaming', 'true'],
        output='screen'
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

    from launch.actions import TimerAction
    delayed_spawn = TimerAction(period=2.0, actions=[spawn_entity])

    return [
        rsp,
        joint_state_publisher_node,
        gz_sim,
        clock_bridge,
        delayed_spawn,
        joystick,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.world'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_sim_planar)
    ])
