import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Define the robot's name and package name
    robot_name = "rusty"
    package_name = "rusty_gz"
    
    # 1. Setup Resource Paths for Gazebo
    # We need to point Gazebo to the directory ABOVE our package share directory 
    # so that model://rusty_description/ meshes can be resolved.
    description_pkg_share = get_package_share_directory('rusty_description')
    install_dir = os.path.join(description_pkg_share, '..')

    # Update GZ_SIM_RESOURCE_PATH (for Gazebo Sim / Harmonic / Ionic)
    # and IGN_GAZEBO_RESOURCE_PATH (for older versions like Fortress)
    # We use SetEnvironmentVariable so it is scoped to this launch process.
    resource_path_action = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[install_dir]
    )

    # 2. Define launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Specify the world file for Gazebo (e.g., empty.sdf)'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.5')
    roll_arg = DeclareLaunchArgument('R', default_value='0.0')
    pitch_arg = DeclareLaunchArgument('P', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0')

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    roll, pitch, yaw = LaunchConfiguration('R'), LaunchConfiguration('P'), LaunchConfiguration('Y')

    # 3. Process Xacro
    robot_model_path = os.path.join(description_pkg_share, 'urdf', 'rusty.urdf.xacro')
    robot_description = xacro.process_file(robot_model_path).toxml()

    gz_bridge_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    # 4. Include Gazebo Launch
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={
            'gz_args': [f'-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 5. Nodes
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-string', robot_description,
            '-x', x, '-y', y, '-z', z,
            '-R', roll, '-P', pitch, '-Y', yaw,
            '-allow_renaming', 'false'
        ],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        output='screen'
    )

    return LaunchDescription([
        resource_path_action, # Ensure environment is set first
        world_arg,
        gazebo_launch,
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,
        spawn_model_gazebo_node,
        robot_state_publisher_node,
        gz_bridge_node,
    ])