// Open loop motor control example
#include <SimpleFOC.h>

namespace OpenVelLoopExample {
  // BLDCMotor(pole pair number, phase resistance (optional) );
  BLDCMotor motor = BLDCMotor(11);
  // BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
  BLDCDriver3PWM driver = BLDCDriver3PWM(4, 5, 6, 0);
  
  // instantiate the commander
  Commander command = Commander(Serial);
  void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
  void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
  
  void setup() {
  
    // use monitoring with serial 
    Serial.begin(115200);
    // enable more verbose output for debugging
    // comment out if not needed
    SimpleFOCDebug::enable(&Serial);
  
    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 19;
    // limit the maximal dc voltage the driver can set
    // as a protection measure for the low-resistance motors
    // this value is fixed on startup
    driver.voltage_limit = 6;
    if(!driver.init()){
      Serial.println("Driver init failed!");
      return;
    }
    // link the motor and the driver
    motor.linkDriver(&driver);
  
    // limiting motor movements
    // limit the voltage to be set to the motor
    // start very low for high resistance motors
    // current = voltage / resistance, so try to be well under 1Amp
    motor.voltage_limit = 3;   // [V]
   
    // open loop control config
    motor.controller = MotionControlType::velocity_openloop;
  
    // init motor hardware
    if(!motor.init()){
      Serial.println("Motor init failed!");
      return;
    }
  
    // set the target velocity [rad/s]
    motor.target = 6.28; // one rotation per second
  
    // add target command T
    command.add('T', doTarget, "target velocity");
    command.add('L', doLimit, "voltage limit");
  
    Serial.println("Motor ready!");
    Serial.println("Set target velocity [rad/s]");
    _delay(1000);
  }
  
  void loop() {
  
    // open loop velocity movement
    motor.move();
  
    // user communication
    command.run();
  }
}