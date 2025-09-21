#include <DynamixelShield.h>

/*
// Mantenha as mesmas configurações de comunicação do seu código principal
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // RX, TX
  #define DXL_SERIAL   soft_serial
#else
  #define DXL_SERIAL   Serial1
#endif

DynamixelShield dxl(DXL_SERIAL, DXL_DIR_PIN);

// IMPORTANTE: Use a mesma lista de IDs que você está tentando usar no seu código principal
const int JOINT_COUNT = 6;
const uint8_t dxl_ids[JOINT_COUNT] = {1, 2, 3, 4, 5, 6}; // <-- Use seus IDs reais aqui

void setup() {
  // Serial para o Monitor Serial do PC
  Serial.begin(115200);
  while (!Serial); // Espera a porta serial conectar

  // Serial para os servos
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(1.0);

  Serial.println("Iniciando teste de leitura dos servos...");
}

void loop() {
  String output = "Posições Atuais: ";
  
  // Lê a posição de cada servo e adiciona na string de saída
  for (int i = 0; i < JOINT_COUNT; i++) {
    int position = dxl.getPresentPosition(dxl_ids[i]);
    output += "ID ";
    output += String(dxl_ids[i]);
    output += ": ";
    output += String(position);
    if (i < JOINT_COUNT - 1) {
      output += ", ";
    }
  }
  
  // Imprime o resultado no Monitor Serial
  Serial.println(output);
  
  delay(500); // Atraso de meio segundo para facilitar a leitura
}

*/
