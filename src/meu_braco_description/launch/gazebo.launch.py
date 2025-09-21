from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():  

    # Caminho para o pacote de descrição do braço
    pkg_description = get_package_share_directory("meu_braco_description")

    # Argumento para o modelo XACRO
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_description, "urdf", "model.xacro"),
        description="Absolute path to robot URDF file"
    )

    # Processa o arquivo XACRO
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Nó do Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    # Inicia o Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items()
    )

    # Insere o robô no Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "arm_6dof"
        ]
    )

    # Spawner para o publicador de estado das juntas
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner para o controlador do BRAÇO
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["braco_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner para o controlador da GARRA
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["garra_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller
    ])