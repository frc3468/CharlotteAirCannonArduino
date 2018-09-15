#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <Servo.h>
#include <RobotOpenHA.h>


/* I/O Setup */
ROJoystick usb1(1);         // Joystick #1

// ARDUINO PIN 10 IS RESERVED FOR THE ETHERNET CONTROLLER!!!!!

ROPWM compressor(0);
RODigitalIO compressorPressureSwitch(1, INPUT_PULLUP); // Low means low pressure

boolean compressorOverride = true;
boolean compressorState = false;
boolean compressorButtonReleased = true;
int compressorTimeout = 10000; // Milliseconds
ROTimer compressorShutoffTimer;

ROPWM leftDriveMotor(2);
ROPWM rightDriveMotor(3);
ROPWM cannonLiftMotor(4);

ROPWM leftCannonSolenoid(5);
ROPWM rightCannonSolenoid(6);

boolean armButtonReleased = true;
int cannonFireTime = 200; // Milliseconds
ROTimer cannonShutoffTimer;

void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);

  compressor.attach();

  compressorShutoffTimer.queue(0);
  leftCannonSolenoid.attach();
  rightCannonSolenoid.attach();

  cannonLiftMotor.attach();

  leftDriveMotor.attach();
  rightDriveMotor.attach();

}


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled() {
  // Motor Axes
  int forwardPower = usb1.leftY();
  int turnPower = map(usb1.leftX(), 0, 255, 255, 0); // Inverted
  int cannonLiftPower = usb1.rightY();

  // ArcadeDrive formulas
  int leftDrivePower = constrain((usb1.leftY() + usb1.leftX()), 0, 255);
  int rightDrivePower = constrain((usb1.leftY() - usb1.leftX()), 0, 255);

  // Command Buttons
  boolean armCannonButton = usb1.btnA();
  boolean leftCannonFireButton = usb1.btnLShoulder();
  boolean rightCannonFireButton = usb1.btnRShoulder();

  // Compressor Control
  if(!compressorOverride) {
    boolean atPressure = compressorPressureSwitch.read();

    // Compressor turns ON when PressureSwitch is OFF.
    if(atPressure) {
      compressor.write(127);
    } else {
      compressor.write(255);
    }

  } else {
    boolean compressorToggleButton = usb1.btnY();
    
    //If pressing toggle and previously released it
    if(compressorToggleButton && compressorButtonReleased) {
      if(compressorState) {
        // Turn Compressor Off
        compressor.write(127);
        compressorState = false;
      } else {
        // Turn Compressor On
        compressor.write(255);
        compressorState = true;
        // Setup Cutoff Timer
        compressorShutoffTimer.queue(compressorTimeout);
      }
      // We need to release the button
      compressorButtonReleased = false;
    }
    if(!compressorToggleButton) {
      // Button has been released
      compressorButtonReleased = true;
    }

    if(compressorShutoffTimer.ready()) {
      // Shutoff Compressor
      compressor.write(127);
      compressorState = false;
    }
  }
  
  // Cannon Control
  if(armCannonButton && armButtonReleased) {
    // Cannon can only fire if arming button has been reset and pressed
    if(leftCannonFireButton) {
      // Fire Left Cannon and setup cutoff
      leftCannonSolenoid.write(255);
      cannonShutoffTimer.queue(cannonFireTime);
      armButtonReleased = false;
    }
    if(rightCannonFireButton) {
      // Fire Left Cannon and setup cutoff
      rightCannonSolenoid.write(255);
      cannonShutoffTimer.queue(cannonFireTime);
      armButtonReleased = false;
    }
    //TODO Move Controls into functions to reduce code re-use
  }
  if(!armCannonButton) {
    // Button has been relased
    armButtonReleased = true;
  }

  // Cannons will shutoff after timer expires
  if(cannonShutoffTimer.ready()) {
    leftCannonSolenoid.write(127);
    rightCannonSolenoid.write(127);
  }

  // Turret Lift Control
  cannonLiftMotor.write(cannonLiftPower);

  // Drive Control
  leftDriveMotor.write(leftDrivePower);
  rightDriveMotor.write(rightDrivePower);
}


/* This is called while the robot is disabled */
void disabled() {
  // safety code

  // Shut-off Compressor
  compressor.write(127);

  // Neutral-Out PWMs
  leftDriveMotor.write(127);
  rightDriveMotor.write(127);
  cannonLiftMotor.write(127);

  // Close Solenoids
  leftCannonSolenoid.write(127);
  rightCannonSolenoid.write(127);

}


/* This loop ALWAYS runs - only place code here that can run during a disabled state
 * This is also a good spot to put driver station publish code
 */
void timedtasks() {
  RODashboard.publish("Uptime Seconds", ROStatus.uptimeSeconds());
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
