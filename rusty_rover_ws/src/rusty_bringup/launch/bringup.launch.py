import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    sim_mode = LaunchConfiguration('sim_mode')
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Use simulation (Gazebo) if true'
    )

    gz_pkg_share =  get_package_share_directory('rusty_gz')
    description_pkg_share = get_package_share_directory('rusty_description')
    
    robot_description_content = Command([
        'xacro ', os.path.join(description_pkg_share, 'urdf', 'rusty.urdf.xacro'),
        ' sim_mode:=', sim_mode
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('rusty_description'),
                    'launch',
                    'rsp.launch.py',
                )
            ]
        ),
        launch_arguments={
            'sim_mode': sim_mode,
        }.items(),
    )

    gz_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('rusty_gz'),
                    'launch',
                    'gz_bringup.launch.py',
                )
            ]
        ),
        launch_arguments={
            'sim_mode': sim_mode,
        }.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('rusty_control'),
                    'launch',
                    'control.launch.py',
                )
            ]
        ),
        launch_arguments={
            'sim_mode': sim_mode,
        }.items(),
    )

    return LaunchDescription([
        sim_mode_arg,
        rsp,
        gz_bringup,
        control,
    ])