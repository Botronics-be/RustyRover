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
    
    world_file = LaunchConfiguration('world', default='empty.sdf')

    gz_bridge_params_path = os.path.join(gz_pkg_share, 'config', 'gz_bridge.yaml')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file]}.items()
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'rusty', '-topic', 'robot_description'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sim_mode_arg,
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[os.path.join(description_pkg_share, '..')]),
        gazebo_launch,
        spawn_model,
        robot_state_publisher,
        gz_bridge,

        # This ensures the spawners only start AFTER the robot is created in Gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_model,
                on_exit=[joint_state_broadcaster_spawner, robot_controller_spawner],
            )
        ),
    ])