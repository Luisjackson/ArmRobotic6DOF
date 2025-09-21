from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config_pkg = get_package_share_directory('meu_braco_moveit_config')

    # Nó do MoveGroup
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # RViz com a configuração do MoveIt
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'moveit_rviz.launch.py')
        )
    )

    return LaunchDescription([
        # Apenas os nós do MoveIt! são iniciados aqui
        move_group_launch,
        moveit_rviz_launch
    ])