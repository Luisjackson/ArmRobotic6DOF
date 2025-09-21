import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm_6dof", package_name="meu_braco_moveit_config")
        .robot_description(file_path="config/arm_6dof.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Nó do ros2_control (inicia primeiro)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                FindPackageShare("meu_braco_moveit_config").find("meu_braco_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="screen",
    )

    # Publicador de estado do robô
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("meu_braco_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Nó do MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Spawners que serão iniciados DEPOIS do ros2_control_node
    spawner_actions = []
    for controller in ["joint_state_broadcaster", "braco_controller", "garra_controller"]:
        spawner_actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # Event handler para iniciar os spawners DEPOIS que o ros2_control_node estiver ativo
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=spawner_actions,
        )
    )

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher_node,
            move_group_node,
            ros2_control_node,
            delayed_spawners,
        ]
    )