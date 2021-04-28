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
const uint8_t driverControllerUSB = 1;

// PWM + Digital
// ARDUINO PIN 10 IS RESERVED FOR THE ETHERNET CONTROLLER!!!!!
const uint8_t leftDriveMotorsPWM = 2;
const uint8_t rightDriveMotorsPWM = 3;
const uint8_t cannonLiftMotorPWM = 4;

const uint8_t compressorRelayDIO = 5;
const uint8_t leftCannonRelayDIO = 6;
const uint8_t rightCannonRelayDIO = 7;

// Analog
const uint8_t voltageDividerAnalog = 0;
const uint8_t pressureTransducerAnalog = 1;


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
int8_t driveSpeedAxis() { return map(driverController.leftY(), 255, 0, -128, 127); /* inverted */ }
int8_t driveRotationAxis() { return map(driverController.leftX(),0, 255, -128, 127); }
const uint8_t driveMinSpeed = 0; // -1.0
const uint8_t driveMaxSpeed = 255; // 1.0

// Compressor Variables
const uint8_t compressorMinPressure = 60; //psi
const uint8_t compressorMaxPressure = 80; //psi
// TODO Impliment Cycle Time Limits(?)

// Cannon Lift Variables
uint8_t cannonLiftAxis() { return map(driverController.rightY(), 255, 0, 0, 255); /* inverted*/ }
const uint8_t cannonLiftMinSpeed = 64; // -0.5
const uint8_t cannonLiftMaxSpeed = 192; // 0.5
// TODO Impliment Limit Switches(?)

// Cannon Variables
bool leftCannonArmButton() { return driverController.btnA(); }
bool rightCannonArmButton() { return driverController.btnB(); }
bool cannonTriggerButton() { return (driverController.lTrigger() == 255); }
const uint16_t cannonChargeTime = 5000; //milliseconds
const uint16_t cannonFireTime = 150; // milliseconds
const uint8_t unarmedCannonID = 0x00;
const uint8_t leftCannonID = 0x01;
const uint8_t rightCannonID = 0x02;
uint8_t cannonsArmed = unarmedCannonID;
uint8_t cannonsLock = unarmedCannonID;
ROTimer cannonChargeTimer;
ROTimer cannonShutoffTimer;


// Voltage Divider Variables
const uint8_t voltageDividerMaxReading = 25; // volts
float voltageDividerStep = (float)voltageDividerMaxReading/ANALOG_RESOLUTION;

// Pressure Transducer Variables
const uint8_t pressureTransducerMaxReading = 150; // psi
float pressureTransducerStep = (float)pressureTransducerMaxReading/ANALOG_RESOLUTION;


void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);  

  leftDriveMotor.attach();
  rightDriveMotor.attach();
  cannonLiftMotor.attach();
}


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled() {
  // Drivetrain Control
  int8_t driveSpeed = driveSpeedAxis();
  int8_t driveRotation = driveRotationAxis();
  
  uint8_t leftDriveSpeed = constrain(map(driveSpeed+driveRotation, -128, 127, 255, 0), driveMinSpeed, driveMaxSpeed);
  uint8_t rightDriveSpeed = constrain(map(driveSpeed-driveRotation, -128, 127, 0, 255), driveMinSpeed, driveMaxSpeed);
  
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

  // Cannon Arming Buttons
  if(leftCannonArmButton()) {
    cannonsArmed = cannonsArmed | leftCannonID;
  } else {
    cannonsArmed = cannonsArmed & ~leftCannonID;
  }
  
  if(rightCannonArmButton()) {
    cannonsArmed = cannonsArmed | rightCannonID;
  } else {
    cannonsArmed = cannonsArmed & ~rightCannonID;
  }

  // Cannon Trigger Pulled, Queue Charge Timer
  if(cannonTriggerButton() && cannonsArmed && !cannonsLock) {
    cannonChargeTimer.queue(cannonChargeTime);
    cannonsLock = cannonsArmed;
  }

  // Cancel Charge Sequence if anything changes after lock set
  if(cannonChargeTimer.isActive()) {
    if(!cannonTriggerButton() || cannonsArmed != cannonsLock) {
      cannonChargeTimer.cancel();
    }
  }

  // Fire the Armed Cannons, Queue Shutoff Timer
  if(cannonChargeTimer.ready()) {
    if(cannonsLock & leftCannonID) {
      leftCannonRelay.on();
    }
    if(cannonsLock & rightCannonID) {
      rightCannonRelay.on();
    }
    cannonShutoffTimer.queue(cannonFireTime);
  }

  // Shutoff Cannons
  if(cannonShutoffTimer.ready()) {
    leftCannonRelay.off();
    rightCannonRelay.off();
  }

  // Reset Arming Lock when Trigger is Released
  if(cannonsLock && !cannonTriggerButton()) {
    cannonsLock = unarmedCannonID;
  }
  
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
