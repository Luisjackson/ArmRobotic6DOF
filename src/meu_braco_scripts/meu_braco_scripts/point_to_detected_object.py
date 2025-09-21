#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

import moveit_commander
from moveit_commander import PlanningSceneInterface

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class PointToObjectNode(Node):
    def __init__(self):
        super().__init__('point_to_object_node')

        self.get_logger().info("Nó iniciado. Pronto para apontar para objetos.")
        
        # MoveIt e ros2_control setup
        moveit_commander.roscpp_initialize([])
        self.scene = PlanningSceneInterface()
        action_name = '/braco_controller/follow_joint_trajectory'
        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)
        self.joint_names = [
            'waist_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'gripper_base_joint'
        ]


    def point_at_target(self, target_name):
        self.get_logger().info(f"Procurando pelo objeto '{target_name}' na cena...")

        # Procurar o objeto
        object_poses = self.scene.get_object_poses([target_name])
        if not object_poses:
            self.get_logger().error(f"Não encontrei o objeto '{target_name}' na cena!")
            return
        
        target_pose = object_poses[target_name]
        target_x = target_pose.position.x
        target_y = target_pose.position.y

        # Calcula o angulo corrige
        base_angle_raw = math.atan2(target_y, target_x)
        corrected_angle = base_angle_raw + math.pi # adicionar 180 graus para orientacao, corrige apontar para lado oposto
        base_angle_normalized = normalize_angle(corrected_angle) # garantir o caminho mais curto

        self.get_logger().info(f"Objeto encontrado em (X={target_x:.2f}, Y={target_y:.2f}). Ângulo final: {base_angle_normalized:.2f} radianos.")

        # enviar a trajetoria
        pointing_positions = [base_angle_normalized, 0.1, 0.1, -1.6, 0.0]

        # enviar o comando Usa a lógica de Action Client
        self._action_client.wait_for_server()
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = pointing_positions
        point.time_from_start.sec = 3
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self.get_logger().info(f"Enviando meta para o braço...")
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointToObjectNode()

    NOME_DO_OBJETO = "Box_0" # OBjeto que vai ser criado no rviz

    # node.get_logger().info(f"vc tem 10 segundos para adicionar o objeto '{NOME_DO_OBJETO}' no RViz...")
    # time.sleep(10) 

    # Apontar para o objeto
    node.point_at_target(NOME_DO_OBJETO)
    time.sleep(5)

    node.get_logger().info("Teste concluído.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()