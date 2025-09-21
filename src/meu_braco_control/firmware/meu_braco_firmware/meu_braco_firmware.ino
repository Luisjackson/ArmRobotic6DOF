#include <DynamixelShield.h>

DynamixelShield dxl;
using namespace ControlTableItem;

const int JOINT_COUNT = 6;

const uint8_t dxl_ids[JOINT_COUNT] = {1, 2, 3, 4, 6, 5};

void setup() {
  // Inicia a comunicação serial com o PC
  Serial.begin(115200);

  // Inicia a comunicação com os servos Dynamixel
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(1.0);

  // Configura cada um dos servos
  for (int i = 0; i < JOINT_COUNT; i++) {
    if(dxl.ping(dxl_ids[i])) {
      dxl.setOperatingMode(dxl_ids[i], OP_POSITION);
      dxl.torqueOn(dxl_ids[i]);
    }
  }
}

void loop() {
  //  RECEBER COMANDOS DO ROS2 E MOVER OS SERVOS
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if (command.startsWith("P")) {
      command.remove(0, 2);
      
      int current_joint_index = 0;
      
      while (command.length() > 0 && current_joint_index < JOINT_COUNT) {
        int spaceIndex = command.indexOf(' ');
        
        String pos_str;
        if (spaceIndex == -1) {
          pos_str = command;
          command = "";
        } else {
          pos_str = command.substring(0, spaceIndex);
          command.remove(0, spaceIndex + 1);
        }
        
        int goal_position = pos_str.toInt();
        
        dxl.setGoalPosition(dxl_ids[current_joint_index], goal_position);
        
        current_joint_index++;
      }
    }
  }

  // LER O ESTADO ATUAL E ENVIAR DE VOLTA PARA O ROS2
  String status_string = "S ";
  for (int i = 0; i < JOINT_COUNT; i++) {
    status_string += String(dxl.getPresentPosition(dxl_ids[i]));
    
    if (i < JOINT_COUNT - 1) {
      status_string += " ";
    }
  }
  Serial.println(status_string);

  delay(50); 
}
