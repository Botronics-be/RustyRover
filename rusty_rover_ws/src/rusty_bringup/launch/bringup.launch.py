from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    sim_mode = LaunchConfiguration('sim_mode')
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        choices=['true', 'false'],
        description='Set to "true" to launch simulation. Set to "false" for physical hardware.'
    )

    pkg_description = FindPackageShare('rusty_description')
    pkg_control     = FindPackageShare('rusty_control')
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_description, 'launch', 'rsp.launch.py'])
        ]),
        launch_arguments={'sim_mode': sim_mode}.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_control, 'launch', 'control.launch.py'])
        ]),
        launch_arguments={'sim_mode': sim_mode}.items(),
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        remappings=[
            ("/image_raw", "/camera/image_raw")
        ],
        condition=UnlessCondition(sim_mode) 
    )

    camera_capture_node = Node(
        package="rusty_camera",
        executable="toggle_publication"
    )

    bluetooth = Node(
        package='rusty_ble_server',
        executable='bluetooth_server',
        name='bluetooth_server_node',
        respawn=True,
        respawn_delay=2,
    )

    command_dispatcher = Node(
        package='rusty_command_dispatcher',
        executable='command_dispatcher',
        name='command_dispatcher',
        respawn=True,
        respawn_delay=2,
    )

    spinner_node = Node(
        package='rusty_command_dispatcher',
        executable='spinner_node',
        name='spinner_node',
        respawn=True,
        respawn_delay=2,
    )

    pilot_node = Node(
        package='rusty_command_dispatcher',
        executable='pilot_node',
        name='pilot_node',
        respawn=True,
        respawn_delay=2,
    )

    return LaunchDescription([
        sim_mode_arg,
        rsp,
        control,
        v4l2_camera_node,
        camera_capture_node,
        bluetooth,
        command_dispatcher,
        spinner_node,
        pilot_node,
    ])