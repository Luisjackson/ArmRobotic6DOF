from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    robot_description_path = os.path.join(
        get_package_share_directory("meu_braco_description"), 
        "urdf",
        "model.xacro"
    )

    robot_description = ParameterValue(
        Command(["xacro ", robot_description_path]),
        value_type=str
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},

            os.path.join(
                get_package_share_directory("meu_braco_control"),
                "config",
                "controllers.yaml"
            )
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Spawner para o controlador que publica o estado das juntas
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Spawner para o controlador de trajetória do braço
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )


    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        # gripper_controller_spawner # Descomentar se fizer futuramente um controlador de garra
    ])