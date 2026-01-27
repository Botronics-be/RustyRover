from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition 
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    sim_mode = LaunchConfiguration("sim_mode")
    
    declared_arguments = [
        DeclareLaunchArgument(
            "sim_mode",
            default_value="true",
            choices=["true", "false"],
            description="Use simulation (Gazebo) or real hardware",
        )
    ]

    pkg_control = FindPackageShare("rusty_control")
    controllers_config_path = PathJoinSubstitution([pkg_control, "config", "controller.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_config_path],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
        condition=UnlessCondition(sim_mode) 
    )

    def create_spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--controller-ros-args",
                "-r /controller_manager/robot_description:=/robot_description",
            ],
            parameters=[{'use_sim_time': sim_mode}],
            output="screen",
        )

    nodes = [
        control_node,
        create_spawner("joint_state_broadcaster"),
        create_spawner("diff_cont"),
    ]

    return LaunchDescription(declared_arguments + nodes)