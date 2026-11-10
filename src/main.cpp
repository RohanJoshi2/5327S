/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       rohanjoshi                                                */
/*    Created:      9/22/2026, 6:03:39 PM                                     */
/*    Description:  6-Motor Arcade Drive + Odometry                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <math.h>

using namespace vex;

brain Brain;

// A global instance of competition
competition Competition;

// Controller
controller Controller1 = controller(primary);

// ---- ODOMETRY CONFIG (TUNE / MEASURE THESE) ----
const double ODOM_WHEEL_DIAM_IN = 2.0;      // tracking wheel diameter (inches)
const double ODOM_DIST_FROM_CENTER_IN = 5.0; // distance from bot center to each tracking wheel (inches) -- MEASURE
const bool LEFT_ODOM_REVERSED = false;       // flip if the sensor reads negative when driving forward
const bool RIGHT_ODOM_REVERSED = false;      // flip if the sensor reads negative when driving forward

// Misc Motors
motor toggleMotor = motor(PORT10, false);
digital_out clawPneumatic = digital_out(Brain.ThreeWirePort.A);
bool clawPneumaticOpen = false;

// Sensors
inertial inertialSensor = inertial(PORT11);
rotation leftOdom = rotation(PORT12, LEFT_ODOM_REVERSED);
rotation rightOdom = rotation(PORT13, RIGHT_ODOM_REVERSED);

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

/*---------------------------------------------------------------------------*/
/*                                 Odometry                                  */
/*---------------------------------------------------------------------------*/
/*
 * Frame: +Y = robot's starting forward direction, +X = starting right.
 * Heading: degrees, 0 = +Y, clockwise positive (matches inertial default).
 *
 * Both tracking wheels point the same way as the drive wheels, sit on the same
 * lateral axis, and are equidistant from the bot center. That means:
 *   - forward travel of the center = average of the two wheels
 *   - turning in place cancels out of that average (no position drift)
 *   - heading change from wheels alone = (right - left) / (2 * distance)
 */

const double ODOM_WHEEL_CIRC_IN = ODOM_WHEEL_DIAM_IN * M_PI;
const double ODOM_IN_PER_DEG = ODOM_WHEEL_CIRC_IN / 360.0;

// Pose shared between the odometry task and everything else
vex::mutex odomMutex;
double odomX = 0;            // inches
double odomY = 0;            // inches
double odomHeadingDeg = 0;   // inertial heading (continuous, not wrapped)
double odomWheelHeadingDeg = 0; // heading estimated from the tracking wheels only

double prevLeftOdomDeg = 0;
double prevRightOdomDeg = 0;
double prevHeadingDeg = 0;

// Heading used by the pose: inertial if it is connected, otherwise wheel-derived
double currentHeadingDeg() {
  if (inertialSensor.installed()) {
    return inertialSensor.rotation(degrees);
  }
  return odomWheelHeadingDeg;
}

// Set the robot pose (x, y in inches, heading in degrees)
void setPose(double x, double y, double headingDeg) {
  odomMutex.lock();
  if (inertialSensor.installed()) {
    inertialSensor.setRotation(headingDeg, degrees);
  }
  odomX = x;
  odomY = y;
  odomHeadingDeg = headingDeg;
  odomWheelHeadingDeg = headingDeg;
  prevLeftOdomDeg = leftOdom.position(degrees);
  prevRightOdomDeg = rightOdom.position(degrees);
  prevHeadingDeg = headingDeg;
  odomMutex.unlock();
}

// Thread-safe pose getters
double getX() {
  odomMutex.lock();
  double v = odomX;
  odomMutex.unlock();
  return v;
}

double getY() {
  odomMutex.lock();
  double v = odomY;
  odomMutex.unlock();
  return v;
}

double getHeading() {
  odomMutex.lock();
  double v = odomHeadingDeg;
  odomMutex.unlock();
  return v;
}

double getWheelHeading() {
  odomMutex.lock();
  double v = odomWheelHeadingDeg;
  odomMutex.unlock();
  return v;
}

// Background task: integrates tracking wheel + inertial data into a pose
int odometryLoop() {
  while (true) {
    double leftDeg = leftOdom.position(degrees);
    double rightDeg = rightOdom.position(degrees);

    double dLeftIn = (leftDeg - prevLeftOdomDeg) * ODOM_IN_PER_DEG;
    double dRightIn = (rightDeg - prevRightOdomDeg) * ODOM_IN_PER_DEG;
    prevLeftOdomDeg = leftDeg;
    prevRightOdomDeg = rightDeg;

    // Heading change from the wheels alone (clockwise positive)
    double dWheelHeadingDeg =
      ((dRightIn - dLeftIn) / (2.0 * ODOM_DIST_FROM_CENTER_IN)) * 180.0 / M_PI;

    // Forward travel of the bot center over this step
    double dForwardIn = (dLeftIn + dRightIn) / 2.0;

    odomMutex.lock();
    odomWheelHeadingDeg += dWheelHeadingDeg;

    double headingDeg = currentHeadingDeg();
    // Use the average heading over the step for a better arc approximation
    double avgHeadingRad = ((prevHeadingDeg + headingDeg) / 2.0) * M_PI / 180.0;
    prevHeadingDeg = headingDeg;

    odomX += dForwardIn * sin(avgHeadingRad);
    odomY += dForwardIn * cos(avgHeadingRad);
    odomHeadingDeg = headingDeg;
    odomMutex.unlock();

    wait(10, msec);
  }
  return 0;
}

/*---------------------------------------------------------------------------*/

// Toggle claw pneumatic
void toggleClaw() {
  clawPneumaticOpen = !clawPneumaticOpen;
  clawPneumatic.set(clawPneumaticOpen);
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

enum direction { UP, DOWN, STOP };

void DR4BFor(direction d, int durationMs) {
  if (d == UP) {
    DR4B(true, false);
  } else if (d == DOWN) {
    DR4B(false, true);
  } else {
    DR4B(false, false);
  }
  wait(durationMs, msec);
  DR4B(false, false);
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

// PID function (distance measured by odometry, heading held by inertial)
void drivePID(double inches) {
  if (inches == 0) return;

  const double wheelCircumference = 2.75 * M_PI;
  // Motor degrees per inch of travel. The PID works in "motor degree" units so the
  // existing kP/kI/kD tuning still applies, but the measurement comes from odometry.
  const double motorDegPerInch = 1.09 * (360.0 / wheelCircumference);
  double targetMotorDeg = inches * motorDegPerInch;

  // Tune ONLY these three values
  double kP = 0.09;
  double kI = 0.0015;
  double kD = 0.001;

  // Heading hold (power percent per degree of heading error)
  const double kHeading = 1.2;
  const double MAX_HEADING_CORRECTION = 15.0;

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

  // Starting pose: distance is measured along the starting heading
  double startX = getX();
  double startY = getY();
  double startHeadingDeg = getHeading();
  double startHeadingRad = startHeadingDeg * M_PI / 180.0;
  double dirX = sin(startHeadingRad);
  double dirY = cos(startHeadingRad);

  while (elapsed < TIMEOUT) {
    // Signed distance traveled along the starting heading (inches) from odometry
    double traveledIn = (getX() - startX) * dirX + (getY() - startY) * dirY;
    double avgPos = traveledIn * motorDegPerInch;

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

    // Smooth velocity to keep derivative damping from reacting to noise.
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

    // Heading correction: positive error = robot is left of target heading
    double headingError = startHeadingDeg - getHeading();
    double correction = kHeading * headingError;
    if (correction > MAX_HEADING_CORRECTION) correction = MAX_HEADING_CORRECTION;
    if (correction < -MAX_HEADING_CORRECTION) correction = -MAX_HEADING_CORRECTION;
    // Fade the correction out as power drops so it can't spin the bot at the end
    correction *= fmin(1.0, fabs(power) / 20.0);

    double leftPower = power + correction;
    double rightPower = power - correction;
    if (leftPower > 100) leftPower = 100;
    if (leftPower < -100) leftPower = -100;
    if (rightPower > 100) rightPower = 100;
    if (rightPower < -100) rightPower = -100;

    leftDrive.spin(fwd, leftPower, pct);
    rightDrive.spin(fwd, rightPower, pct);

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
  // Calibrate the inertial sensor (keep the robot still)
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(20, msec);
  }
  clawPneumatic.set(false);

  // Start the pose at the origin facing +Y
  setPose(0, 0, 0);
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  drivePID(12);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  bool previousX = false;
  while (1) {
    bool currentX = Controller1.ButtonX.pressing();
    if (currentX && !previousX) {
      toggleClaw();
    }
    previousX = currentX;

    arcadeDrive();
    DR4B(Controller1.ButtonR1.pressing(), Controller1.ButtonR2.pressing());
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

  // Start odometry after calibration so the pose starts clean
  task odomTask = task(odometryLoop);

  while (true) {
    wait(100, msec);
  }
}
