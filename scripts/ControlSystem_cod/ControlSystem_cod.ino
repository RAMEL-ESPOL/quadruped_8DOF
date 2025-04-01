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
//Si sale este mensaje: "MOT: PP check: fail - estimated pp" el numeronde polos esta mal
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(INHA, INHB, INHC, EN_GATE);

Encoder sensor = Encoder(2, 3, 5000);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){
  sensor.handleA();
  }
void doB(){
  sensor.handleB();
  }


/* 
HallA and B are on the arduino uno hardware intterrupt pins which are pin 2 and 3.
because the arduino uno only has 2 hardware intterrupt pins Hall C needs to be
placed on a software intterupt pin.
this is what this does.
*/


// voltage set point variable
float target_voltage = 1;
// instantiate the commander


void setup() { 
  
  // initialize Hall sensor hardware
  sensor.pullup = Pullup::USE_EXTERN;
  sensor.quadrature = Quadrature::OFF;
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
  driver.voltage_limit = 24;
  driver.pwm_frequency = 32000;
  driver.init(); //start driver
  motor.linkDriver(&driver); // link driver
  motor.voltage_sensor_align = 1;   // aligning voltage limit
  
  //motor mode selection
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //set torque mode:
  motor.torque_controller = TorqueControlType::voltage;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; 
  // downsampling
  motor.monitor_downsample = 10; // default 10
  
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();



  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);
  //motor.monitor();

}
