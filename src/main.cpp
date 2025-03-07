/**
 * 
 * Torque control example using voltage control loop.
 * 
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead hte current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */
#include <SimpleFOC.h>


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(4, 5, 6, 0);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// encoder instance
Encoder encoder = Encoder(17, 16, 2048);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// voltage set point variable
float target = 20;
// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() { 
  
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  //motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE
  motor.monitor_downsample = 10; // default 10

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 19;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  motor.voltage_limit = 4;
  motor.velocity_limit = 20;
  //motor.LPF_angle.Tf = 5;
  // aligning voltage
  motor.voltage_sensor_align = 2;
  //encoder.min_elapsed_time=0.01; //2ms
  motor.motion_downsample = 50;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;  // Reduced from 20
  motor.PID_velocity.D = 0.0;  // Increased slightly


  motor.LPF_velocity.Tf = 0.02;  // Increased from 0.1

  motor.controller = MotionControlType::velocity;

  // comment out if not needed
  //motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  //command.add('T', doTarget, "target voltage");
  command.add('M',doMotor,"my motor");
  _delay(1000);
}

void loop() {

 // motor.PID_velocity.I = target;
  motor.loopFOC();
  motor.move();
  motor.monitor();
  command.run();
}