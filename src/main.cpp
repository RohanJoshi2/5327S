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
motor clawMotor = motor(PORT7, true);
motor clawLiftMotor = motor(PORT15, false);

// Lift motors
motor liftLeft = motor(PORT8, false);
motor liftRight = motor(PORT9, true);

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
bool clawLiftActive = false;
void clawLiftToggle() {

  clawLiftMotor.setVelocity(100, percent);

  if (clawLiftActive) {
    clawLiftMotor.spinFor(reverse, 325, degrees);
  } else {
    clawLiftMotor.spinFor(forward, 325, degrees);
  }

  clawLiftActive = !clawLiftActive;
  clawLiftMotor.stop(hold);
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
void drivePID(double targetMotorDeg) {
  double kP = 0.1;
  double kI = 0.0004;
  double kD = 0.6;

  double error = 0;
  double prevError = 0;
  double integral = 0;
  double derivative = 0;

  const int LOOP_TIME = 20; // ms
  const int TIMEOUT = 2000; // ms
  int elapsed = 0;

  leftMotor1.setPosition(0, degrees);
  leftMotor2.setPosition(0, degrees);
  leftMotor3.setPosition(0, degrees);
  rightMotor1.setPosition(0, degrees);
  rightMotor2.setPosition(0, degrees);
  rightMotor3.setPosition(0, degrees);

  while (elapsed < TIMEOUT) {

    double leftPos =
      (leftMotor1.position(degrees) +
       leftMotor2.position(degrees) +
       leftMotor3.position(degrees)) / 3.0;

    double rightPos =
      (rightMotor1.position(degrees) +
       rightMotor2.position(degrees) +
       rightMotor3.position(degrees)) / 3.0;

    double avgPos = (leftPos + rightPos) / 2.0;

    error = targetMotorDeg - avgPos;

    if (fabs(error) < 2) break;

    if (fabs(error) < 50) {
      integral += error;
    } else {
      integral = 0;
    }

    if (integral > 3000) integral = 3000;
    if (integral < -3000) integral = -3000;

    derivative = error - prevError;
    prevError = error;

    double power =
      (kP * error) +
      (kI * integral) +
      (kD * derivative);

    if (power > 100) power = 100;
    if (power < -100) power = -100;

    leftDrive.spin(fwd, power, pct);
    rightDrive.spin(fwd, power, pct);

    wait(LOOP_TIME, msec);
    elapsed += LOOP_TIME;
  }

  leftDrive.stop(coast);
  rightDrive.stop(coast);
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
    bool currentA = Controller1.ButtonA.pressing();

    if (currentA && !previousA) {
      clawLiftToggle();
    }

    previousA = currentA;

    arcadeDrive();
    intake(Controller1.ButtonR1.pressing(), Controller1.ButtonR2.pressing());
    lift(Controller1.ButtonL1.pressing(), Controller1.ButtonL2.pressing());

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