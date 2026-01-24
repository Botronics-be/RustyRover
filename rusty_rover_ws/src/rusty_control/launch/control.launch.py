from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition # Added this
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "sim_mode",
            default_value="true",
            description="Use simulation (Gazebo) or real hardware",
        )
    ]

    sim_mode = LaunchConfiguration("sim_mode")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rusty_description"), "urdf", "rusty.urdf.xacro"]
            ),
            " ",
            "sim_mode:=",
            sim_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rusty_control"),
            "config",
            "controller.yaml",
        ]
    )

    # --- THE BRAIN ---
    # This node is ONLY launched for the real robot.
    # Gazebo provides its own version of this node via the plugin.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        condition=UnlessCondition(sim_mode) # Magic line
    )

    # --- THE CONTROLLERS ---
    # These run in both modes. They wait for a controller_manager to appear.
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    nodes = [
        control_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)