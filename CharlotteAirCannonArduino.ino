#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <Servo.h>
#include <RobotOpenHA.h>
//TODO WS2812B LED Strip Drive

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


// Compressor Variables
uint8_t compressorMinPressure = 60; //psi
uint8_t compressorMaxPressure = 80; //psi

// Cannon Lift Variables
uint8_t cannonLiftMinSpeed = 64; // -0.5
uint8_t cannonLiftMaxSpeed = 192; // 0.5
//TODO Impliment Limit Switches(?)


// Voltage Divider Variables
uint8_t voltageDividerMinReading = 0; // volts
uint8_t voltageDividerMaxReading = 25; // volts

// Pressure Transducer Variables
uint8_t pressureTransducerMinReading = 0; // psi
uint8_t pressureTransducerMaxReading = 150; // psi


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
  uint8_t driveSpeed = driverController.leftY();
  uint8_t driveRotation = map(driverController.leftX(), 255, 0, 0, 255); // inverted

  uint8_t leftDrivePower = constrain((driveSpeed + driveRotation), 0, 255);
  uint8_t rightDrivePower = constrain((driveSpeed - driveRotation), 0, 255);

  leftDriveMotor.write(leftDrivePower);
  rightDriveMotor.write(rightDrivePower);


  // Cannon Lift Control
  uint8_t cannonLiftSpeed = constrain(driverController.rightY(),cannonLiftMinSpeed,cannonLiftMaxSpeed);
  
  cannonLiftMotor.write(cannonLiftSpeed);

  
  // Compressor Control
  uint8_t pressure = map(pressureTransducer.read(), 0, 1023, pressureTransducerMinReading, pressureTransducerMaxReading);
  
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
  leftDriveMotor.write(127);
  rightDriveMotor.write(127);
  cannonLiftMotor.write(127);

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
  RODashboard.publish("Voltage (v)", map(voltageDivider.read(), 0, 1023, voltageDividerMinReading, voltageDividerMaxReading));
  RODashboard.publish("Pressure (psi)", map(pressureTransducer.read(), 0, 1023, pressureTransducerMinReading, pressureTransducerMaxReading));
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
