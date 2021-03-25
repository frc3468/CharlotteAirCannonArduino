/*
#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <Servo.h>
*/

#include <RobotOpenHA.h>
//TODO WS2812B LED Strip Drive

#define ANALOG_RESOLUTION 1024 // 10-bit
#define PWM_NEUTRAL 127 // 0.0

// Controllers
uint8_t driverControllerUSB = 1;

// PWM + Digital
// ARDUINO PIN 10 IS RESERVED FOR THE ETHERNET CONTROLLER!!!!!
uint8_t leftDriveMotorsPWM = 2;
uint8_t rightDriveMotorsPWM = 3;
uint8_t cannonLiftMotorPWM = 4;

uint8_t compressorRelayDIO = 5;
uint8_t leftCannonRelayDIO = 6;
uint8_t rightCannonRelayDIO = 7;

// Analog
uint8_t voltageDividerAnalog = 0;
uint8_t pressureTransducerAnalog = 1;


// I/O Objects
ROJoystick driverController(driverControllerUSB);

ROPWM leftDriveMotor(leftDriveMotorsPWM);
ROPWM rightDriveMotor(rightDriveMotorsPWM);
ROPWM cannonLiftMotor(cannonLiftMotorPWM);

RODigitalIO compressorRelay(compressorRelayDIO, OUTPUT);
RODigitalIO leftCannonRelay(leftCannonRelayDIO, OUTPUT);
RODigitalIO rightCannonRelay(rightCannonRelayDIO, OUTPUT);

ROAnalog voltageDivider(voltageDividerAnalog);
ROAnalog pressureTransducer(pressureTransducerAnalog);

// Drivetrain Variables
uint8_t driveSpeedAxis() { return map(driverController.leftY(), 255, 0, 0, 255); /* inverted*/ }
uint8_t driveRotationAxis() { return driverController.leftX(); }
uint8_t driveMinSpeed = 0; // -1.0
uint8_t driveMaxSpeed = 255; // 1.0

// Compressor Variables
uint8_t compressorMinPressure = 60; //psi
uint8_t compressorMaxPressure = 80; //psi
// TODO Impliment Cycle Time Limits(?)

// Cannon Lift Variables
uint8_t cannonLiftAxis() { return map(driverController.rightY(), 255, 0, 0, 255); /* inverted*/ }
uint8_t cannonLiftMinSpeed = 64; // -0.5
uint8_t cannonLiftMaxSpeed = 192; // 0.5
// TODO Impliment Limit Switches(?)

// Cannon Variables
bool leftCannonArmedButton() { return driverController.btnA(); }
bool rightCannonArmedButton() { return driverController.btnB(); }
bool cannonTriggerButton() { return (driverController.lTrigger() == 255); }


// Voltage Divider Variables
uint8_t voltageDividerMaxReading = 25; // volts
float voltageDividerStep = (float)voltageDividerMaxReading/ANALOG_RESOLUTION;

// Pressure Transducer Variables
uint8_t pressureTransducerMaxReading = 150; // psi
float pressureTransducerStep = (float)pressureTransducerMaxReading/ANALOG_RESOLUTION;


void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);  
}


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled() {
  // Drivetrain Control
  uint8_t driveSpeed = driveSpeedAxis();
  uint8_t driveRotation = driveRotationAxis();
  
  uint8_t leftDriveSpeed = constrain((driveSpeed + driveRotation), driveMinSpeed, driveMaxSpeed);
  uint8_t rightDriveSpeed = constrain((driveSpeed - driveRotation), driveMinSpeed, driveMaxSpeed);

  leftDriveMotor.write(leftDriveSpeed);
  rightDriveMotor.write(rightDriveSpeed);


  // Cannon Lift Control
  uint8_t cannonLiftSpeed = constrain(cannonLiftAxis(), cannonLiftMinSpeed, cannonLiftMaxSpeed);
  cannonLiftMotor.write(cannonLiftSpeed);

  
  // Compressor Control
  float pressure = pressureTransducer.read()*pressureTransducerStep;
  
  if(pressure >= compressorMaxPressure) {
    compressorRelay.off();
    RODashboard.publish("Compressor", true);
  } else if(pressure <= compressorMinPressure) {
    compressorRelay.on();
    RODashboard.publish("Compressor", false);
  }


  // TODO Cannon Control State Machine
  
}


/* This is called while the robot is disabled */
void disabled() {
  // safety code

  // Neutral-Out PWMs
  leftDriveMotor.write(PWM_NEUTRAL);
  rightDriveMotor.write(PWM_NEUTRAL);
  cannonLiftMotor.write(PWM_NEUTRAL);

  // Close Solenoids
  leftCannonRelay.off();
  rightCannonRelay.off();

  // Turn off Compressor
  compressorRelay.off();
  RODashboard.publish("Compressor", false);

}


/* This loop ALWAYS runs - only place code here that can run during a disabled state
 * This is also a good spot to put driver station publish code
 */
void timedtasks() {
  RODashboard.publish("Uptime (s)", ROStatus.uptimeSeconds());
  RODashboard.publish("Voltage (v)", (voltageDivider.read()*voltageDividerStep));
  RODashboard.publish("Pressure (psi)", (pressureTransducer.read()*pressureTransducerStep));
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
