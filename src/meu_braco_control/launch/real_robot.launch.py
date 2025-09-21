from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Argumento para a porta serial
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyACM0"
    )

    #  Descrição do Robô para o HARDWARE REAL 
    robot_description_pkg = get_package_share_directory("meu_braco_description")
    robot_xacro_file = os.path.join(robot_description_pkg, "urdf", "model.xacro")
    robot_control_file = os.path.join(robot_description_pkg, "urdf", "real_robot.ros2_control.xacro")
    robot_description_content = Command([
        "xacro ", robot_xacro_file, " ",
        "ros2_control_xacro_path:=", robot_control_file, " ",
        "serial_port:=", LaunchConfiguration("serial_port")
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Nó do ros2_control 
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory("meu_braco_moveit_config"), "config", "ros2_controllers.yaml")
        ],
        output="screen",
    )

    #Nó do Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawners dos controladores 
    spawners = []
    for controller in ["joint_state_broadcaster", "braco_controller", "garra_controller"]:
        spawners.append(Node(package="controller_manager", executable="spawner", arguments=[controller]))

    # Garante que os spawners só iniciem depois do controller_manager
    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=spawners,
        )
    )

    return LaunchDescription([
        serial_port_arg,
        ros2_control_node,
        robot_state_publisher_node,
        delayed_spawners,
    ])