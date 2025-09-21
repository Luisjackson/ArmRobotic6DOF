#!/usr/bin/env python3

import sys
import rclpy
import moveit_commander
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
import time

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.logger = self.get_logger()
        
        # Inicialização do MoveIt Commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Interfaces principais
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("braco")
        self.gripper_group = moveit_commander.MoveGroupCommander("garra")
        
        self.end_effector_link = self.arm_group.get_end_effector_link()
        self.arm_group.set_planning_time(10.0) # Aumenta o tempo para planejamento

        self.logger.info("Nó de Pick and Place iniciado.")

    def add_collision_objects(self):
        """Adiciona a mesa e o objeto a ser pego na cena."""
        self.scene.remove_world_object("table")
        self.scene.remove_world_object("box_to_pick")
        
        # Adiciona a mesa na FRENTE do robô
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.robot.get_planning_frame()
        table_pose.pose.position.x = 0.4  # Posição X na frente
        table_pose.pose.position.z = 0.2
        self.scene.add_box("table", table_pose, size=(0.5, 1.0, 0.4))

        # Adiciona a caixa na FRENTE do robô, em cima da mesa
        box_pose = PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = -0.2
        box_pose.pose.position.z = 0.45 # Acima da mesa (0.4 de altura / 2)
        self.scene.add_box("box_to_pick", box_pose, size=(0.04, 0.04, 0.1))
        
        self.logger.info("Mesa e caixa adicionadas à cena.")
        time.sleep(1)

    def set_gripper(self, state):
        """Abre ('open') ou fecha ('closed') a garra."""
        joint_values = self.gripper_group.get_current_joint_values()
        if state == "open":
            joint_values[0] = 0.0 
            self.logger.info("Abrindo a garra.")
        elif state == "closed":
            joint_values[0] = 0.8
            self.logger.info("Fechando a garra.")
        
        self.gripper_group.go(joint_values, wait=True)
        self.gripper_group.stop()

    def go_to_pose(self, pose):
        """Move o braço para uma pose específica."""
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def perform_pick(self):
        self.logger.info("Iniciando sequência de 'pick' manual.")
        
        box_pose = self.scene.get_object_poses(["box_to_pick"])["box_to_pick"]
        
        # Mover para uma posição de pré-aproximação
        pre_grasp_pose = Pose()
        pre_grasp_pose.orientation.w = 1.0
        pre_grasp_pose.position.x = box_pose.position.x
        pre_grasp_pose.position.y = box_pose.position.y
        pre_grasp_pose.position.z = box_pose.position.z + 0.15
        if not self.go_to_pose(pre_grasp_pose): return False
        
        # Abrir a garra
        self.set_gripper("open")
        
        # Mover para pegar (aproximação final)
        grasp_pose = pre_grasp_pose
        grasp_pose.position.z -= 0.13 # Desce 13cm
        if not self.go_to_pose(grasp_pose): return False
        
        # Fechar a garra
        self.set_gripper("closed")
        
        # Anexar a caixa à garra
        self.arm_group.attach_object("box_to_pick", self.end_effector_link)
        self.logger.info("Caixa anexada.")
        
        # Recuar
        retreat_pose = pre_grasp_pose # Volta para a posição segura acima
        if not self.go_to_pose(retreat_pose): return False
        
        return True

    def perform_place(self):
        self.logger.info("Iniciando sequência de 'place' manual.")
        
        # Mover para uma posição de largada
        place_pose = Pose()
        place_pose.orientation.w = 1.0
        place_pose.position.x = 0.4
        place_pose.position.y = 0.2 # Local de largada
        place_pose.position.z = 0.55 # Um pouco mais alto para evitar colisão
        if not self.go_to_pose(place_pose): return False
             
        # Abrir a garra e desanexar o objeto
        self.set_gripper("open")
        self.arm_group.detach_object("box_to_pick")
        self.logger.info("Caixa desanexada.")

        return True

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    
    node.add_collision_objects()
    
    node.arm_group.set_named_target("inicial")
    node.arm_group.go(wait=True)
    
    if node.perform_pick():
        node.perform_place()
    
    node.get_logger().info("Tarefa concluída.")
    rclpy.shutdown()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()