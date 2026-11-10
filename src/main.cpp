/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       rohanjoshi                                                */
/*    Created:      9/22/2026, 6:03:39 PM                                     */
/*    Description:  6-Motor Arcade Drive                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// Controller
controller Controller1 = controller(primary);

// Misc Motors
motor toggleMotor = motor(PORT10, false);
digital_out clawPneumatic = digital_out(brain.three_wire_port.a);
bool clawPneumaticOpen = false;

// DR4B motors
motor DR4BLeft = motor(PORT7, true);
motor DR4BRight = motor(PORT9, false);

// Left side
motor leftMotor1 = motor(PORT4, ratio6_1, true);   // 5.5W
motor leftMotor2 = motor(PORT5, ratio6_1, false);   // 11W
motor leftMotor3 = motor(PORT6, ratio6_1, false);   // 11W

// Right side
motor rightMotor1 = motor(PORT1, ratio6_1, false);   // 5.5W
motor rightMotor2 = motor(PORT2, ratio6_1, true);   // 11W
motor rightMotor3 = motor(PORT3, ratio6_1, true);   // 11W

// Motor groups
motor_group DR4BMotors = motor_group(DR4BLeft, DR4BRight);
motor_group leftDrive = motor_group(leftMotor1, leftMotor2, leftMotor3);
motor_group rightDrive = motor_group(rightMotor1, rightMotor2, rightMotor3);

//

// Pneumatic controls
void setClawPneumatic(bool open) {
  clawPneumaticOpen = open;
  clawPneumatic.set(clawPneumaticOpen);
}

void toggleClawPneumatic() {
  setClawPneumatic(!clawPneumaticOpen);
}

// Intake / mechanism toggle
void intake(bool in, bool out) {
  if (in && !out) {
    toggleMotor.spin(forward, 100, percent);
  } else if (out && !in) {
    toggleMotor.spin(reverse, 100, percent);
  } else {
    toggleMotor.stop(hold);
  }
}

void startClaw(bool in) {
  if (in) {
    toggleMotor.spin(forward, 100, percent);
  } else {
    toggleMotor.spin(reverse, 100, percent);
  }
}

void stopClaw() {
  toggleMotor.stop(hold);
}

void intakeFor(bool in, bool out, int durationMs) {
  intake(in, out);
  wait(durationMs, msec);
  intake(false, false);
}

// DR4B
void DR4B(bool up, bool down) {
  if (up && !down) {
    DR4BMotors.spin(forward, 100, percent);
  } else if (down && !up) {
    DR4BMotors.spin(reverse, 100, percent);
  } else {
    DR4BMotors.stop(hold);
  }
}

void DR4BFor(bool up, bool down, int durationMs) {
  DR4B(up, down);
  wait(durationMs, msec);
  DR4B(false, false);
}

void turnFor(bool left, bool right, int durationMs) {
  if (left && !right) {
    leftDrive.spin(reverse, 100, percent);
    rightDrive.spin(forward, 100, percent);
  } else if (right && !left) {
    leftDrive.spin(forward, 100, percent);
    rightDrive.spin(reverse, 100, percent);
  } else {
    leftDrive.stop(brake);
    rightDrive.stop(brake);
  }
  wait(durationMs, msec);
  leftDrive.stop(brake);
  rightDrive.stop(brake);
}

// ---- TUNE THESE 3 VALUES ----
const double TRACK_WIDTH_IN = 11.5;   // distance between left/right wheels (inches)
const double WHEEL_DIAM_IN  = 2.75;   // matches drivePID wheel diameter
const double MOTOR_PER_WHEEL = 1.09;  // matches drivePID encoder multiplier
const double TURN_RATE_CORRECTION = 0.75; // calibrate from measured turn angle

// Convert robot turn degrees -> motor degrees (per side) for turn-in-place
double turnDegToMotorDeg(double robotDeg) {
  // each wheel travels an arc whose radius is TRACK_WIDTH/2
  double arcLen = (robotDeg * M_PI / 180.0) * (TRACK_WIDTH_IN / 2.0); // inches

  double wheelCirc = WHEEL_DIAM_IN * M_PI;                            // inches per wheel rev
  double wheelRevs = arcLen / wheelCirc;                              // wheel revs
  double motorRevs = wheelRevs * MOTOR_PER_WHEEL;                     // motor revs

  return motorRevs * 360.0 * TURN_RATE_CORRECTION;                    // corrected motor degrees
}

void turnClockwise(int deg) {
  double targetMotorDeg = fabs(turnDegToMotorDeg(deg));
  if (targetMotorDeg == 0) return;

  const int LOOP_TIME_MS = 10;
  const int STALL_TIMEOUT_MS = 500;
  const double STALL_MOVEMENT_DEG = 2.0;
  int leftStallMs = 0;
  int rightStallMs = 0;
  double lastLeftProgress = 0;
  double lastRightProgress = 0;

  // reset encoders (motor_group can't reset)
  leftMotor1.setPosition(0, vex::deg);
  leftMotor2.setPosition(0, vex::deg);
  // leftMotor3.setPosition(0, vex::deg);
  // rightMotor1.setPosition(0, vex::deg);
  rightMotor2.setPosition(0, vex::deg);
  rightMotor3.setPosition(0, vex::deg);

  int leftSign = (deg > 0) ? -1 : 1;
  int rightSign = -leftSign;

  while (true) {
    double leftPos =
      (fabs(leftMotor1.position(vex::deg)) +
       fabs(leftMotor2.position(vex::deg))) / 3.0;
    double rightPos =
      (fabs(rightMotor2.position(vex::deg)) +
       fabs(rightMotor3.position(vex::deg))) / 3.0;

    bool leftDone = leftPos >= targetMotorDeg;
    bool rightDone = rightPos >= targetMotorDeg;
    if (leftDone && rightDone) break;

    if (leftDone || leftPos - lastLeftProgress >= STALL_MOVEMENT_DEG) {
      leftStallMs = 0;
      lastLeftProgress = leftPos;
    } else {
      leftStallMs += LOOP_TIME_MS;
    }
    if (rightDone || rightPos - lastRightProgress >= STALL_MOVEMENT_DEG) {
      rightStallMs = 0;
      lastRightProgress = rightPos;
    } else {
      rightStallMs += LOOP_TIME_MS;
    }
    if (leftStallMs >= STALL_TIMEOUT_MS || rightStallMs >= STALL_TIMEOUT_MS) break;

    if (leftDone) {
      leftDrive.stop(vex::brake);
    } else {
      leftDrive.spin(vex::fwd, 50 * leftSign, vex::pct);
    }
    if (rightDone) {
      rightDrive.stop(vex::brake);
    } else {
      rightDrive.spin(vex::fwd, 50 * rightSign, vex::pct);
    }

    wait(LOOP_TIME_MS, vex::msec);
  }

  leftDrive.stop(vex::brake);
  rightDrive.stop(vex::brake);
  wait(100, vex::msec);
}

// Arcade drive function
void arcadeDrive() {
  int throttle = 0.7*Controller1.Axis3.position();
  int turn = 0.7*Controller1.Axis1.position();

  int leftPower = throttle - turn;
  int rightPower = throttle + turn;

  // Limit the calculated values to the motor's valid range
  if (leftPower > 100)
    leftPower = 100;

  if (leftPower < -100)
    leftPower = -100;

  if (rightPower > 100)
    rightPower = 100;

  if (rightPower < -100)
    rightPower = -100;

  leftDrive.spin(forward, leftPower, percent);
  rightDrive.spin(forward, rightPower, percent);
}

// PID function
void drivePID(double inches) {
  const double wheelCircumference = 2.75 * M_PI;
  double targetMotorDeg = 1.09 * ((inches / wheelCircumference) * 360.0);

  // Tune ONLY these three values
  double kP = 0.09;
  double kI = 0.0015;
  double kD = 0.001;

  double error = 0;
  double prevPos = 0;
  double integral = 0;
  double derivative = 0;

  const double LOOP_TIME = 20.0; // ms
  const int TIMEOUT = 2000; // ms
  const double MAX_DRIVE_POWER = 70.0;
  const double MAX_POWER_CHANGE = 2.5; // percentage points per loop
  const double POWER_FILTER_ALPHA = 0.25;
  int elapsed = 0;
  int settledCount = 0;
  double previousPower = 0;
  double filteredPower = 0;

  leftMotor1.setPosition(0, degrees);
  leftMotor2.setPosition(0, degrees);
  leftMotor3.setPosition(0, degrees);
  rightMotor1.setPosition(0, degrees);
  rightMotor2.setPosition(0, degrees);
  rightMotor3.setPosition(0, degrees);

  while (elapsed < TIMEOUT) {
    double leftPos = (leftMotor1.position(degrees) +
                      leftMotor2.position(degrees) +
                      leftMotor3.position(degrees)) / 3.0;

    double rightPos = (rightMotor1.position(degrees) +
                       rightMotor2.position(degrees) +
                       rightMotor3.position(degrees)) / 3.0;

    double avgPos = (leftPos + rightPos) / 2.0;

    error = targetMotorDeg - avgPos;

    // Settling check: must stay within 2 degrees for 5 consecutive loops (100ms)
    if (fabs(error) < 2.0) {
      settledCount++;
    } else {
      settledCount = 0;
    }
    if (settledCount >= 5) break;

    // Integral accumulation & anti-windup capping
    if (fabs(error) < 40.0) {
      integral += error;
    } else {
      integral = 0;
    }
    if (integral > 3000) integral = 3000;
    if (integral < -3000) integral = -3000;

    // Smooth encoder velocity to keep derivative damping from reacting to noise.
    double measuredVelocity = (avgPos - prevPos) / (LOOP_TIME / 1000.0);
    derivative += 0.25 * (measuredVelocity - derivative);
    prevPos = avgPos;

    double power = (kP * error) + (kI * integral) - (kD * derivative);

    // Power capping
    if (power > MAX_DRIVE_POWER) power = MAX_DRIVE_POWER;
    if (power < -MAX_DRIVE_POWER) power = -MAX_DRIVE_POWER;

    // Minimum power deadband to prevent motor stalling/humming
    if (fabs(power) < 3.0) {
      power = 0;
    }

    filteredPower += POWER_FILTER_ALPHA * (power - filteredPower);
    power = filteredPower;

    double powerChange = power - previousPower;
    if (powerChange > MAX_POWER_CHANGE) {
      power = previousPower + MAX_POWER_CHANGE;
    } else if (powerChange < -MAX_POWER_CHANGE) {
      power = previousPower - MAX_POWER_CHANGE;
    }
    previousPower = power;

    leftDrive.spin(fwd, power, pct);
    rightDrive.spin(fwd, power, pct);

    wait(LOOP_TIME, msec);
    elapsed += LOOP_TIME;
  }

  // Passive electromagnetic braking mode (protects motors from thermal strain)
  leftDrive.stop(brake);
  rightDrive.stop(brake);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  setClawPneumatic(false);
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Tuning
  // drivePID(12);

  // Toggle auton
  // startClaw(false);
  // drivePID(10);
  // clawLiftFor(true, false, 425);
  // leftDrive.spin (reverse, 100, percent);
  // rightDrive.spin(reverse, 100, percent);
  // wait(340, msec);
  // leftDrive.stop(coast);
  // rightDrive.stop(coast);
  // drivePID(10);
  // stopClaw();

  //Side auton
  // startClaw(true);
  // drivePID(8);
  // clawLiftFor(true, false, 475);
  // leftDrive.spin(reverse, 100, percent);
  // rightDrive.spin(reverse, 100, percent);
  // wait(300, msec);
  // leftDrive.stop(coast);
  // rightDrive.stop(coast);
  // drivePID(11);
  // clawLiftFor(false, true, 230);
  // turnClockwise(90);
  // drivePID(-6);
  // stopClaw();
  // startClaw(false);
  // wait(200, msec);
  // stopClaw();
  // wait(200, msec);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  bool previousX = false;

  while (1) {
    bool currentX = Controller1.ButtonX.pressing();

    if (currentX && !previousX) {
      toggleClawPneumatic();
    }
    previousX = currentX;

    arcadeDrive();
    DR4BFor(Controller1.ButtonR1.pressing(), Controller1.ButtonR2.pressing(), 100);
    wait(20, msec);
  }
}

/*---------------------------------------------------------------------------*/
/*                                Main                                        */
/*---------------------------------------------------------------------------*/

int main() {

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}