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
motor intakeMotor = motor(PORT11, false);
motor clawMotor = motor(PORT10, true);
motor clawLiftMotor = motor(PORT17, false);

// Lift motors
motor liftLeft = motor(PORT9, true);
motor liftRight = motor(PORT8, false);

// Left side
motor leftMotor1 = motor(PORT1, ratio6_1, false);   // 11W
motor leftMotor2 = motor(PORT2, ratio6_1, false);   // 11W
motor leftMotor3 = motor(PORT3, ratio6_1, false);   // 5.5W

// Right side
motor rightMotor1 = motor(PORT4, ratio6_1, true);   // 11W
motor rightMotor2 = motor(PORT5, ratio6_1, true);   // 11W
motor rightMotor3 = motor(PORT6, ratio6_1, true);   // 5.5W

// Motor groups
motor_group liftMotors = motor_group(liftLeft, liftRight);
motor_group leftDrive = motor_group(leftMotor1, leftMotor2, leftMotor3);
motor_group rightDrive = motor_group(rightMotor1, rightMotor2, rightMotor3);

//

// Intake
void intake(bool in, bool out) {
  if (in && !out) {
    intakeMotor.spin(forward, 100, percent);
    clawMotor.spin(forward, 100, percent);
  } else if (out && !in) {
    intakeMotor.spin(reverse, 100, percent);
    clawMotor.spin(reverse, 100, percent);
  } else {
    intakeMotor.stop(coast);
    clawMotor.stop(hold);
  }
}

void intakeFor(bool in, bool out, int durationMs) {
  intake(in, out);
  wait(durationMs, msec);
  intake(false, false);
}

// Lift
void lift(bool up, bool down) {
  if (up && !down) {
    liftMotors.spin(forward, 100, percent);
  } else if (down && !up) {
    liftMotors.spin(reverse, 100, percent);
  } else {
    liftMotors.stop(hold);
  }
}

void liftFor(bool up, bool down, int durationMs) {
  lift(up, down);
  wait(durationMs, msec);
  lift(false, false);
}

// Claw Lift
void clawLift(bool up, bool down) {
  if (up && !down) {
    clawLiftMotor.spin(forward, 100, percent);
  } else if (down && !up) {
    clawLiftMotor.spin(reverse, 100, percent);
  } else {
    clawLiftMotor.stop(hold);
  }
}

void clawLiftFor(bool up, bool down, int durationMs) {
  clawLift(up, down);
  wait(durationMs, msec);
  clawLift(false, false);
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

// bool clawLiftActive = false;
// void clawLiftToggle() {

//   clawLiftMotor.setVelocity(100, percent);

//   if (clawLiftActive) {
//     clawLiftMotor.setVelocity(75, percent);
//     clawLiftMotor.spinFor(reverse, 300, degrees);
//     clawLiftMotor.setVelocity(100, percent);
//   } else {
//     clawLiftMotor.spinFor(forward, 300, degrees);
//   }

//   clawLiftActive = !clawLiftActive;
//   clawLiftMotor.stop(hold);
// }

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
  double kP = 0.08;
  double kI = 0.001;
  double kD = 0.0015;

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
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  bool previousA = false;

  while (1) {
    // bool currentA = Controller1.ButtonA.pressing();

    // if (currentA && !previousA) {
    //   clawLiftToggle();
    // }

    // previousA = currentA;

    arcadeDrive();
    intake(Controller1.ButtonL1.pressing(), Controller1.ButtonL2.pressing());
    lift(Controller1.ButtonR1.pressing(), Controller1.ButtonR2.pressing());
    clawLift(Controller1.ButtonA.pressing(), Controller1.ButtonB.pressing());
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