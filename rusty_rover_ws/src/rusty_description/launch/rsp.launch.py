import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    sim_mode = LaunchConfiguration('sim_mode')

    sim_mode_arg = DeclareLaunchArgument(
    'sim_mode',
    default_value='true',
    description='Use simulation (Gazebo) if true'
    )
    
    description_pkg_share = get_package_share_directory('rusty_description')
    
    robot_description_content = Command([
        'xacro ', os.path.join(description_pkg_share, 'urdf', 'rusty.urdf.xacro'),
        ' sim_mode:=', sim_mode
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': sim_mode}],
        output='screen'
    )

    return LaunchDescription([
        sim_mode_arg,
        robot_state_publisher
    ])