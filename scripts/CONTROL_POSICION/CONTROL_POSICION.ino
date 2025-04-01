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
#define INHA   2
#define INHB   3
#define INHC   4

//InH PWM pins
#define INHA2   5
#define INHB2   6
#define INHC2   7
//miscellaneous pins
#define   M_FAULT_PIN 22
#define   EN_GATE 23
#define   M_PWM 28 
#define   M_OC 29
#define   OC_ADJ 24

#define   M_FAULT_PIN2 48
#define   EN_GATE2 46
#define   M_PWM2 50 
#define   M_OC2 51
#define   OC_ADJ2 52
//phase resistance value of the motor A B and C wires.
//float RPhase = 1.4;

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11,0.086,380);
BLDCMotor motor2 = BLDCMotor(11,0.086,380);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(INHA, INHB, INHC, EN_GATE);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(INHA2, INHB2, INHC2, EN_GATE2);

Encoder sensor = Encoder(18, 19, 5000 );
Encoder sensor2 = Encoder(20, 21, 5000 );
float angulos[] = {-25,25,-25,25,-25};
// Interrupt routine intialisation
// channel A and B callbacks
void doA(){
  sensor.handleA();
  }
void doB(){
  sensor.handleB();
  }

void doA2(){
  sensor2.handleA();
  }
void doB2(){
  sensor2.handleB();
  }
/* 
HallA and B are on the arduino uno hardware intterrupt pins which are pin 2 and 3.
because the arduino uno only has 2 hardware intterrupt pins Hall C needs to be
placed on a software intterupt pin.
this is what this does.
*/


// voltage set point variable
float target_angle  = 0
;
float target_angle2  = 0;

Commander command = Commander(Serial);
void onTarget(char* cmd){ command.scalar(&target_angle, cmd); }
void onTarget2(char* cmd){ command.scalar(&target_angle2, cmd); }
void setup() { 
  
  // initialize Hall sensor hardware
  sensor.pullup = Pullup::USE_EXTERN;
  //simplefoc activate sensor
  sensor.init();
  sensor.enableInterrupts(doA, doB);

  sensor2.pullup = Pullup::USE_EXTERN;
  //simplefoc activate sensor
  sensor2.init();
  sensor2.enableInterrupts(doA2, doB2);

// commander interface



  // link the motor to the sensor
  motor.linkSensor(&sensor);

  motor2.linkSensor(&sensor2);

  // driver config
  //DRV802 driver specific code
  //3 or 6 PWM put on high for 3 pwm driver
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);
  pinMode(M_PWM2, OUTPUT);
  digitalWrite(M_PWM2, HIGH);
  //over current protection
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);
  pinMode(M_FAULT_PIN, INPUT);
  pinMode(OC_ADJ, OUTPUT); //adjuster voltage limit for M_OC
  digitalWrite(OC_ADJ, HIGH); // for now put on high to give maximum over current limit but better to connect it with a pot meter.
  pinMode(M_OC2, OUTPUT);
  digitalWrite(M_OC2, LOW);
  pinMode(M_FAULT_PIN2, INPUT);
  pinMode(OC_ADJ2, OUTPUT); //adjuster voltage limit for M_OC
  digitalWrite(OC_ADJ2, HIGH); // for now put on high to give maximum over current limit but better to connect it with a pot meter.


  driver.voltage_power_supply = 24; // power supply voltage [V]
  driver.voltage_limit = 23;
  motor.current_limit = 15;
  driver.init(); //start driver
  motor.linkDriver(&driver); // link driver

  driver2.voltage_power_supply = 24; // power supply voltage [V]
  driver2.voltage_limit = 23;
  motor2.current_limit = 15;
  driver2.init(); //start driver
  motor2.linkDriver(&driver2); // link driver


  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //set torque mode:
  // set control loop type to be used
  //motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 0.5;
  motor2.PID_velocity.P = 0.5;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  motor2.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01;
  motor2.LPF_velocity.Tf = 0.01;

// setting the limits
// either voltage
// angle PID controller 
// default P=20
motor.P_angle.P = 20; 
motor.P_angle.I = 0;  // usually only P controller is enough 
motor.P_angle.D = 0;  // usually only P controller is enough 

motor2.P_angle.P = 20; 
motor2.P_angle.I = 0;  // usually only P controller is enough 
motor2.P_angle.D = 0; 
// acceleration control using output ramp
// this variable is in rad/s^2 and sets the limit of acceleration
motor.P_angle.output_ramp = 1000; // default 1e6 rad/s^2
motor2.P_angle.output_ramp = 550;

// angle low pass filtering
// default 0 - disabled  
// use only for very noisy position sensors - try to avoid and keep the values very small
motor.LPF_angle.Tf = 0.01f; // default 0
motor2.LPF_angle.Tf = 0.01f;
// setting the limits
//  maximal velocity of the position control
motor.velocity_limit = 5; // rad/s - default 20
motor2.velocity_limit = 5;
  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
motor.useMonitoring(Serial);
motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;

motor2.useMonitoring(Serial);
motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;

motor.monitor_downsample = 10; // default 10
motor.voltage_sensor_align = 1; // default 10
motor2.monitor_downsample = 10; // default 10
motor2.voltage_sensor_align = 1; // default 10

command.add('T', onTarget, "target position");
command.add('W', onTarget2, "target position");
// initialize motor
motor.init();
motor2.init();
// align sensor and start FOC
motor.initFOC();
motor2.initFOC();
_delay(1000);
}


void loop() {
    // each one second
  if (digitalRead(M_FAULT_PIN) == LOW) {
    Serial.println("Fault detected during runtime!");
    return;  // Detener cualquier operación hasta resolver el fallo
  }
  motor.loopFOC();
  motor2.loopFOC();
  command.run();
  motor.move(target_angle2);
  motor2.move(target_angle);
  
  if (sensor.getVelocity() == 0.0) {
    //Actualiza el angulo al que desea estar
    //target_angle = target_angle +0.4;
  }

  if ( (sensor2.getSensorAngle() >= 9.00)  and (sensor2.getSensorAngle() <= 9.05)) {
    //Actualiza el angulo al que desea estar
    //target_angle = 2;
  }

  if ( (sensor2.getSensorAngle() >= 2.00)  and (sensor2.getSensorAngle() <= 2.05)) {
    //Actualiza el angulo al que desea estar
   //target_angle = 9;
  }

   //command1.run();
}
