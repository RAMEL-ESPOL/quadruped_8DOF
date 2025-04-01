/**
 * 
 * Torque control example using voltage control loop.
 * 
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead of the current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */

  #include <SimpleFOC.h>



//InH PWM pins
#define INHA   9
#define INHB   10
#define INHC   11
//miscellaneous pins
#define   EN_GATE 8
#define   M_PWM 6 
#define   M_OC 5
#define   OC_ADJ 7
//phase resistance value of the motor A B and C wires.
//float RPhase = 1.4;

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(INHA, INHB, INHC, EN_GATE);

Encoder sensor = Encoder(2, 3, 5000 );

// Interrupt routine intialisation
// channel A and B c allbacks
void doA(){
  sensor.handleA();
  }
void doB(){
  sensor.handleB();
  }


void setup() {
  // put your setup code here, to run once:

  // initialize Hall sensor hardware
  sensor.pullup = Pullup::USE_EXTERN;
  //simplefoc activate sensor
  sensor.init();
  sensor.enableInterrupts(doA, doB);

   // link the motor to the sensor
  motor.linkSensor(&sensor);
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

  driver.voltage_power_supply = 24; // power supply voltage [V]
  driver.init(); //start driver
  motor.linkDriver(&driver); // link driver

  // set the torque control type
  motor.phase_resistance = 0.24; // 12.5 Ohms
  motor.torque_controller = TorqueControlType::voltage;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  _delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:

  motor.loopFOC();

  motor.move(1);
 //motor.monitor();


}
