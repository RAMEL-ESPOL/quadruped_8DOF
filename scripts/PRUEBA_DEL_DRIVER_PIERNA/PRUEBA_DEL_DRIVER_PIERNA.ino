#include <SimpleFOC.h>

// Pines de conexión
#define RESET_PIN 8
#define FAULT_PIN 6
#define OC_ADJ 5

// BLDCDriver3PWM(pwmA, pwmB, pwmC, (en optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);

void setup() {
  // Configurar pines de RESET y FAULT
  pinMode(RESET_PIN, OUTPUT);
  pinMode(FAULT_PIN, INPUT);
  pinMode(OC_ADJ, OUTPUT); //adjuster voltage limit for M_OC
  digitalWrite(OC_ADJ, HIGH); // for now put on high to give maximum over current limit but better to connect it with a pot meter.

  // Asegúrate de habilitar el controlador
  digitalWrite(RESET_PIN, HIGH);
  
  // Verificar el estado de nFAULT
  if (digitalRead(FAULT_PIN) == LOW) {
    Serial.println("Fault detected: Check power supply, connections, or phases.");
    while (1);
  }
  
  // Configuración del driver
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;

  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }

  driver.enable();
  Serial.println("Driver ready!");
  _delay(1000);
}

void loop() {
  if (digitalRead(FAULT_PIN) == LOW) {
    Serial.println("Fault detected during runtime!");
    return;  // Detener cualquier operación hasta resolver el fallo
  }
  // OJO, NO SE PUED ECONFIGURAR MAS DE DOS FASES CON VOLTAJE,
  // SI LO HACES FALLARA Y EMOEZARA A ENCENDERSE EL LED DE NFAULT
  // Aplicar PWM a otra fase
  driver.setPwm(1, 2, 3);  // Esperar ~6V en la fase B
  delay(1000);
}
