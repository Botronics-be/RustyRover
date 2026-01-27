from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
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
        description='Set to "true" to launch Gazebo. Set to "false" for physical hardware.'
    )

    pkg_description = FindPackageShare('rusty_description')
    pkg_gz          = FindPackageShare('rusty_gz')
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

    bluetooth = Node(
        package='rusty_ble_server',
        executable='bluetooth_server',
        name='bluetooth_server_node',
        respawn=True,
        respawn_delay=2,
    )

    return LaunchDescription([
        sim_mode_arg,
        rsp,
        bluetooth,
        # Small delay to ensure we have our robot set before our controllers
        TimerAction(
            period=2.0,
            actions=[control]
        )
    ])