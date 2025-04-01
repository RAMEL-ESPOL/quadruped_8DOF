//DESCRIPCION DEL CODIGO
//ESTE CODIGO SIRVE PARA EVALUAR EL FUNCIONAMIENTO DEL DRIVER, LA SALIDA DE LAS FASES 
// SERA HABILITADA CUANDO EL DRIVER LLEGUE A LA ALIMENTACION MINIMA, EN ESTE CASO SE LO ALIMENTA CON 
// 24  V 

// BLDC driver standalone example
#include <SimpleFOC.h>

//InH PWM pins
#define INHA   9
#define INHB   10
#define INHC   11
//miscellaneous pins
#define En_Gate  8 //Enable gate driver and current shunt amplifiers.
#define   M_PWM 6 
#define   M_OC 5
#define   OC_ADJ 7

// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(INHA, INHB, INHC, En_Gate);

void setup() {

  Serial.begin(115200);
  // driver config
  //DRV802 driver specific code
  //3 or 6 PWM put on high for 3 pwm driver
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);
  //over current protection
  
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);
  pinMode(OC_ADJ, OUTPUT); //adjuster voltage limit for M_OC
  digitalWrite(OC_ADJ, HIGH); // for now put on high to give maximum over current limit but better to connect it with a pot meter.
  

  // pwm frequency to be used [Hz]
  // for atmega328 fixed to 32kHz
  // esp32/stm32/teensy configurable
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 24;


  // driver init
  driver.init();
Serial.print("Driver init ");
// init driver
if (driver.init())  Serial.println("success!");
else{
  Serial.println("failed!");
  return;
}

  // enable driver
  driver.enable();

  _delay(1000);
}

void loop() {
    // setting pwm
    // phase A: 3V
    // phase B: 6V
    // phase C: 5V
    driver.setPwm(3,7,3);
}