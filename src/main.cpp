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

// Left side
motor leftMotor1 = motor(PORT1, ratio6_1, false);   // 11W
motor leftMotor2 = motor(PORT2, ratio6_1, false);   // 11W
motor leftMotor3 = motor(PORT3, false);             // 5.5W

// Right side
motor rightMotor1 = motor(PORT4, ratio6_1, true);   // 11W
motor rightMotor2 = motor(PORT5, ratio6_1, true);   // 11W
motor rightMotor3 = motor(PORT6, true);             // 5.5W

// Motor groups
motor_group leftDrive = motor_group(leftMotor1, leftMotor2, leftMotor3);
motor_group rightDrive = motor_group(rightMotor1, rightMotor2, rightMotor3);

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

  leftMotor1.setPosition(0, deg);
  leftMotor2.setPosition(0, deg);
  leftMotor3.setPosition(0, deg);
  rightMotor1.setPosition(0, deg);
  rightMotor2.setPosition(0, deg);
  rightMotor3.setPosition(0, deg);

  while (elapsed < TIMEOUT) {

    double leftPos =
      (leftMotor1.position(deg) +
       leftMotor2.position(deg) +
       leftMotor3.position(deg)) / 3.0;

    double rightPos =
      (rightMotor1.position(deg) +
       rightMotor2.position(deg) +
       rightMotor3.position(deg)) / 3.0;

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

  leftDrive.stop(hold);
  rightDrive.stop(hold);
}

// Arcade drive function
void arcadeDrive() {
  int throttle = Controller1.Axis3.position();
  int turn = Controller1.Axis1.position();

  int leftPower = throttle + turn;
  int rightPower = throttle - turn;

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

  while (1) {
    arcadeDrive();

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