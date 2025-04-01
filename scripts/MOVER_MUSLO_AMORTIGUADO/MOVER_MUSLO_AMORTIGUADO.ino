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
#define M_INHA   9
#define M_INHB   10
#define M_INHC   11

#define L_INHA   5
#define L_INHB   6
#define L_INHC   7
//miscellaneous pins
#define   M_FAULT_PIN 6
#define   M_EN_GATE 8
#define   M_PWM 12 
#define   M_OC 4
#define   M_OC_ADJ 5

#define   L_EN_GATE 47
#define   L_PWM 12 
#define   L_OC 4
#define   L_OC_ADJ 5
//phase resistance value of the motor A B and C wires.
//float RPhase = 1.4;

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
//BLDCMotor muslo = BLDCMotor(11);
BLDCMotor muslo = BLDCMotor(11,0.086f,380);
//BLDCMotor pierna = BLDCMotor(11);
//BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM m_driver = BLDCDriver3PWM(M_INHA, M_INHB, M_INHC, M_EN_GATE);
//BLDCDriver3PWM l_driver = BLDCDriver3PWM(L_INHA, L_INHB, L_INHC, L_EN_GATE);

Encoder m_sensor = Encoder(2, 3, 5000 );
//Encoder l_sensor = Encoder(20, 21, 5000 );

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){
  m_sensor.handleA();
  }
void doB(){
  m_sensor.handleB();
  }

//void doC(){
  //l_sensor.handleA();
  //}
//void doD(){
  //l_sensor.handleB();
  //}


/* 
HallA and B are on the arduino uno hardware intterrupt pins which are pin 2 and 3.
because the arduino uno only has 2 hardware intterrupt pins Hall C needs to be
placed on a software intterupt pin.
this is what this does.
*/


// voltage set point variable
float target_velocity  =6;


void setup() { 
  
  // initialize Hall m_sensor hardware
  m_sensor.pullup = Pullup::USE_EXTERN;
  //simplefoc activate m_sensor
  m_sensor.init();
  m_sensor.enableInterrupts(doA, doB);

  //l_sensor.pullup = Pullup::USE_EXTERN;
  //simplefoc activate m_sensor
  //l_sensor.init();
  //l_sensor.enableInterrupts(doC, doD);

  // link the motor to the m_sensor
  muslo.linkSensor(&m_sensor);

  //pierna.linkSensor(&l_sensor);

  // driver config
  //DRV802 driver specific code
  //3 or 6 PWM put on high for 3 pwm driver
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);
  pinMode(M_FAULT_PIN, INPUT);
  //over current protection
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);
  pinMode(M_OC_ADJ, OUTPUT); //adjuster voltage limit for M_OC
  digitalWrite(M_OC_ADJ, HIGH); // for now put on high to give maximum over current limit but better to connect it with a pot meter.

  //pinMode(L_PWM, OUTPUT);
  //digitalWrite(L_PWM, HIGH);
  //over current protection
  //pinMode(L_OC, OUTPUT);
  //digitalWrite(L_OC, LOW);
  //pinMode(L_OC_ADJ, OUTPUT); //adjuster voltage limit for M_OC
  //digitalWrite(L_OC_ADJ, HIGH); // for now put on high to give maximum over current limit but better to connect it with a pot meter.
  if (digitalRead(M_FAULT_PIN) == LOW) {
    Serial.println("Fault detected: Check power supply, connections, or phases.");
    while (1);
  }
  m_driver.voltage_power_supply = 24; // power supply voltage [V]
  m_driver.voltage_limit = 23;
  muslo.current_limit = 15;
  m_driver.init(); //start driver
  muslo.linkDriver(&m_driver); // link driver
 
  //l_driver.voltage_power_supply = 24; // power supply voltage [V]
  //l_driver.init(); //start driver
  //pierna.linkDriver(&l_driver); // link driver
  //motor mode selection
  // choose FOC modulation (optional)
  muslo.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //pierna.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //set torque mode:
  // set control loop type to be used
  muslo.torque_controller = TorqueControlType::voltage;
  muslo.controller = MotionControlType::torque;

  //pierna.torque_controller = TorqueControlType::voltage;
  //pierna.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  muslo.useMonitoring(Serial);
  muslo.monitor_variables = _MON_VEL | _MON_CURR_Q | _MON_CURR_D ;
  // downsampling
  muslo.monitor_downsample = 10; // default 10
  muslo.voltage_sensor_align = 1;  

  //pierna.useMonitoring(Serial);
  //pierna.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; 
  // downsampling
  //pierna.monitor_downsample = 10; // default 10
  
  // initialize motor
  muslo.init();
  // align m_sensor and start FOC
   muslo.initFOC();

  // initialize motor
  //pierna.init();
  // align m_sensor and start FOC
  //pierna.initFOC();
  _delay(1000);
}


void loop() {
  if (digitalRead(M_FAULT_PIN) == LOW) {
    Serial.println("Fault detected during runtime!");
    return;  // Detener cualquier operación hasta resolver el fallo
  }
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  muslo.loopFOC();
  muslo.monitor();
  //Serial.println(muslo.voltage.q);
  //Serial.println(muslo.voltage.d);
  //pierna.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  muslo.move(target_velocity );
  //pierna.move(target_velocity );


}
