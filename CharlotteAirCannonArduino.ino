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
// ARDUINO PINS 0 & 1 ARE RESERVED FOR THE SERIAL INTERFACE!!!
// ARDUINO PIN 10 IS RESERVED FOR THE ETHERNET CONTROLLER!!!!!
const uint8_t leftDriveMotorsPWM = 2;
const uint8_t rightDriveMotorsPWM = 3;
const uint8_t cannonLiftMotorPWM = 4;

const uint8_t leftCannonRelayDIO = 5;
const uint8_t rightCannonRelayDIO = 6;
const uint8_t compressorRelayDIO = 7;

// Analog
const uint8_t pressureTransducerAnalog = 0;
const uint8_t voltageDividerAnalog = 1;


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
const int8_t driveMinSpeed = -96;
const int8_t driveMaxSpeed = 96;

// Compressor Variables
bool compressorRaiseSetpointButton() { return driverController.dPadUp(); }
bool compressorLowerSetpointButton() { return driverController.dPadDown(); }
bool compressorCutoffButton() { return driverController.btnRShoulder(); }
const uint8_t compressorPressureSetpointStep = 5; //psi
const uint8_t compressorPressureMin = 20; //psi
const uint8_t compressorPressureMax = 70; //psi
const uint8_t compressorDebounceMagnitude = 50;
const uint16_t compressorDebounceTime = 10000; // milliseconds
bool compressorRaiseSetpointButtonLastState = false;
bool compressorLowerSetpointButtonLastState = false;
uint8_t compressorPressureSetpoint = 60; //psi
uint8_t compressorDebounceCounter = 0;
bool compressorCutoffButtonLastState = false;
bool compressorCutoff = false;
ROTimer compressorShutoffTimer;

// TODO Impliment Cycle Time Limits(?)

// Cannon Lift Variables
uint8_t cannonLiftAxis() { return map(driverController.rightY(), 0, 255, 0, 255); }
const uint8_t cannonLiftMinSpeed = 64; // -0.5
const uint8_t cannonLiftMaxSpeed = 192; // 0.5
// TODO Impliment Limit Switches(?)
//     Actually, these could be automatic via the Spark Motor Controller.
//     Just need to wire them NO with the GND Pin to the LMT Switches headers

// Cannon Variables
bool leftCannonArmButton() { return driverController.btnA(); }
bool rightCannonArmButton() { return driverController.btnB(); }
bool cannonTriggerButton() { return (driverController.lTrigger() == 255); }
const uint16_t cannonChargeTime = 2500; //milliseconds
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
const float voltageDividerStep = (float)voltageDividerMaxReading/ANALOG_RESOLUTION;

// Pressure Transducer Variables
const uint8_t pressureTransducerMaxReading = 150; // psi
const float pressureTransducerStep = (float)pressureTransducerMaxReading/ANALOG_RESOLUTION;


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

  // Joystick Deadzones
  if(abs(driveSpeed) <= 10) {
    driveSpeed = 0;
  }
  if(abs(driveRotation) <= 10) {
    driveRotation = 0;
  }
  
  uint8_t leftDriveSpeed = map(constrain(driveSpeed+driveRotation, driveMinSpeed, driveMaxSpeed), -128, 127, 255, 0);
  uint8_t rightDriveSpeed = map(constrain(driveSpeed-driveRotation, driveMinSpeed, driveMaxSpeed), -128, 127, 0, 255);
  
  leftDriveMotor.write(leftDriveSpeed);
  rightDriveMotor.write(rightDriveSpeed);


  // Cannon Lift Control
  uint8_t cannonLiftSpeed = constrain(cannonLiftAxis(), cannonLiftMinSpeed, cannonLiftMaxSpeed);
  cannonLiftMotor.write(cannonLiftSpeed);
  
  // Compressor Control
  float pressure = pressureTransducer.read()*pressureTransducerStep;

  if(!compressorCutoff) {
    if(pressure <= compressorPressureSetpoint) {
      compressorRelay.on();
      compressorDebounceCounter = 0;
    } else if(!compressorShutoffTimer.isActive() && ++compressorDebounceCounter >= compressorDebounceMagnitude) {
      compressorShutoffTimer.queue(compressorDebounceTime);
    }

    if(compressorShutoffTimer.ready()) {
      compressorRelay.off();
      compressorDebounceCounter = 0;
    }
  } else {
    compressorRelay.off();
    compressorDebounceCounter = 0;
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
    cannonsLock = cannonsArmed;
    cannonChargeTimer.queue(cannonChargeTime);
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

}


/* This loop ALWAYS runs - only place code here that can run during a disabled state
 * This is also a good spot to put driver station publish code
 */
void timedtasks() {
  // Compressor Setpoint Control
  if(compressorRaiseSetpointButton() && !compressorRaiseSetpointButtonLastState) {
    compressorPressureSetpoint += compressorPressureSetpointStep;
    if(compressorPressureSetpoint > compressorPressureMax) {
      compressorPressureSetpoint = compressorPressureMax;
    }
  }
  
  if(compressorLowerSetpointButton() && !compressorLowerSetpointButtonLastState) {
    compressorPressureSetpoint -= compressorPressureSetpointStep;
    if(compressorPressureSetpoint < compressorPressureMin) {
      compressorPressureSetpoint = compressorPressureMin;
    }
  }

  compressorRaiseSetpointButtonLastState = compressorRaiseSetpointButton();
  compressorLowerSetpointButtonLastState = compressorLowerSetpointButton();


  // Compressor Cutoff Control
  if(compressorCutoffButton() && !compressorCutoffButtonLastState) {
    compressorCutoff = !compressorCutoff;
  }
  compressorCutoffButtonLastState = compressorCutoffButton();

  
  RODashboard.publish("Uptime (s)", ROStatus.uptimeSeconds());
  RODashboard.publish("Batt Voltage (v)", (voltageDivider.read()*voltageDividerStep));
  RODashboard.publish("Pressure (psi)", (pressureTransducer.read()*pressureTransducerStep));
  RODashboard.publish("Target Pressure (psi)", compressorPressureSetpoint);
  
  RODashboard.publish("Compressor State", compressorRelay.read());
  RODashboard.publish("Compressor Killswitch", compressorCutoff);
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
