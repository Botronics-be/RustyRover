from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition 
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
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

    rusty_description_share = FindPackageShare("rusty_description")
    rusty_control_share = FindPackageShare("rusty_control")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([rusty_description_share, "urdf", "rusty.urdf.xacro"]),
            " sim_mode:=", sim_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [rusty_control_share, "config", "controller.yaml"]
    )

    # 3. Nodes
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        condition=UnlessCondition(sim_mode) 
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': sim_mode}] 
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{'use_sim_time': sim_mode}]
    )

    nodes = [
        control_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)