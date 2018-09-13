#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <Servo.h>
#include <RobotOpenHA.h>


/* I/O Setup */
ROJoystick usb1(1);         // Joystick #1

// ARDUINO PIN 9 IS RESERVED FOR THE ETHERNET CONTROLLER!!!!!

ROPWM compressor(0);
RODigitalIO compressorPressureSwitch(1, INPUT_PULLUP); // Low means low pressure

boolean compressorOverride = true;
boolean compressorState = false;
boolean compressorButtonReleased = true;
int compressorTimeout = 12000; // Milliseconds
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
  // ArcadeDrive formulas
  int leftDrivePower = constrain((usb1.leftY() + usb1.leftX()), 0, 255);
  int rightDrivePower = constrain((usb1.leftY() - usb1.leftX()), 0, 255);

  boolean armCannonButton = usb1.btnA();
  boolean leftCannonFireButton = usb1.btnLShoulder();
  boolean rightCannonFireButton = usb1.btnRShoulder();
  int cannonLiftPower = usb1.rightY();

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
    
    if(compressorToggleButton && compressorButtonReleased) {
      if(compressorState) {
        compressor.write(127);
        compressorState = false;
      } else {
        compressor.write(255);
        compressorState = true;
        compressorShutoffTimer.queue(compressorTimeout);
      }
      compressorButtonReleased = false;
    }
    if(!compressorToggleButton) {
      compressorButtonReleased = true;
    }

    if(compressorShutoffTimer.ready()) {
      compressor.write(127);
      compressorState = false;
    }
  }
  
  // Cannon Control
  if(armCannonButton && armButtonReleased) {
    // Cannon will fire if the "ARMING" button has been re-pressed and 
    // if the cannon has not fired for the set amount of time
    if(leftCannonFireButton) {
      leftCannonSolenoid.write(255);
      cannonShutoffTimer.queue(cannonFireTime);
      armButtonReleased = false;
    }
    if(rightCannonFireButton) {
      rightCannonSolenoid.write(255);
      cannonShutoffTimer.queue(cannonFireTime);
      armButtonReleased = false;
    }
  }
  if(!armCannonButton) {
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
