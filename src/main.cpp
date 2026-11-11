/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       rohanjoshi                                                */
/*    Created:      9/22/2026, 6:03:39 PM                                     */
/*    Description:  6-Motor Arcade Drive + Single-Wheel Odometry             */
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
const double ODOM_WHEEL_DIAM_IN = 2.0;          // tracking wheel diameter (inches)

// Single tracking wheel (ACTIVE)
const double ODOM_OFFSET_FROM_CENTER_IN = -5.0; // MEASURE. Signed: + = wheel is RIGHT of bot center, - = LEFT
const bool ODOM_REVERSED = false;               // flip if the sensor reads negative when driving forward

// Two tracking wheels (DISABLED)
// const double ODOM_DIST_FROM_CENTER_IN = 5.0; // distance from bot center to each tracking wheel (inches)
// const bool LEFT_ODOM_REVERSED = false;       // flip if the sensor reads negative when driving forward
// const bool RIGHT_ODOM_REVERSED = false;      // flip if the sensor reads negative when driving forward

// Misc Motors
motor toggleMotor = motor(PORT10, false);
digital_out clawPneumatic = digital_out(Brain.ThreeWirePort.A);
bool clawPneumaticOpen = false;

// Sensors
inertial inertialSensor = inertial(PORT11);
rotation odomWheel = rotation(PORT12, ODOM_REVERSED); // single tracking wheel (ACTIVE)

// Two tracking wheels (DISABLED)
// rotation leftOdom = rotation(PORT12, LEFT_ODOM_REVERSED);
// rotation rightOdom = rotation(PORT13, RIGHT_ODOM_REVERSED);

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
/*                      Odometry (single tracking wheel)                     */
/*---------------------------------------------------------------------------*/
/*
 * Frame: +Y = robot's starting forward direction, +X = starting right.
 * Heading: degrees, 0 = +Y, clockwise positive (matches inertial default).
 *
 * One tracking wheel points the same way as the drive wheels. It cannot measure
 * heading by itself, so the inertial sensor provides heading. Because the wheel
 * is off-center, turning drags it along an arc, so each step removes that arc:
 *     forward travel of center = wheel travel + offset * heading change (radians)
 * (offset is + for a wheel right of center, - for left). Turning in place then
 * adds no position drift, as long as ODOM_OFFSET_FROM_CENTER_IN is accurate.
 */

const double ODOM_WHEEL_CIRC_IN = ODOM_WHEEL_DIAM_IN * M_PI;
const double ODOM_IN_PER_DEG = ODOM_WHEEL_CIRC_IN / 360.0;

// Pose shared between the odometry task and everything else
mutex odomMutex;
double odomX = 0;            // inches
double odomY = 0;            // inches
double odomHeadingDeg = 0;   // inertial rotation (continuous, not wrapped)

double prevOdomDeg = 0;
double prevHeadingDeg = 0;

// Set the robot pose (x, y in inches, heading in degrees)
void setPose(double x, double y, double headingDeg) {
  odomMutex.lock();
  inertialSensor.setRotation(headingDeg, degrees);
  odomX = x;
  odomY = y;
  odomHeadingDeg = headingDeg;
  prevOdomDeg = odomWheel.position(degrees);
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

// Background task: integrates tracking wheel + inertial data into a pose
int odometryLoop() {
  while (true) {
    odomMutex.lock();

    double wheelDeg = odomWheel.position(degrees);
    double headingDeg = inertialSensor.rotation(degrees);

    double dWheelIn = (wheelDeg - prevOdomDeg) * ODOM_IN_PER_DEG;
    double dHeadingRad = (headingDeg - prevHeadingDeg) * M_PI / 180.0;

    // Remove the arc the off-center wheel sweeps while the robot turns
    double dForwardIn = dWheelIn + ODOM_OFFSET_FROM_CENTER_IN * dHeadingRad;

    // Use the average heading over the step for a better arc approximation
    double avgHeadingRad = ((prevHeadingDeg + headingDeg) / 2.0) * M_PI / 180.0;

    odomX += dForwardIn * sin(avgHeadingRad);
    odomY += dForwardIn * cos(avgHeadingRad);
    odomHeadingDeg = headingDeg;

    prevOdomDeg = wheelDeg;
    prevHeadingDeg = headingDeg;
    odomMutex.unlock();

    wait(10, msec);
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
/*                  Odometry (two tracking wheels) -- DISABLED               */
/*---------------------------------------------------------------------------*/
// Kept for reference. To switch back: uncomment this block and the two-wheel
// config/sensor lines above, then comment out the single-wheel versions
// (odomWheel, ODOM_OFFSET_FROM_CENTER_IN, ODOM_REVERSED, and the single-wheel
// setPose, getX, getY, getHeading, odometryLoop and pose variables).
//
// Frame: +Y = robot's starting forward direction, +X = starting right.
// Heading: degrees, 0 = +Y, clockwise positive (matches inertial default).
//
// Both tracking wheels point the same way as the drive wheels, sit on the same
// lateral axis, and are equidistant from the bot center. That means:
//   - forward travel of the center = average of the two wheels
//   - turning in place cancels out of that average (no position drift)
//   - heading change from wheels alone = (left - right) / (2 * distance)
//     (clockwise turn: left wheel goes forward, right wheel goes backward)
//
// // Pose shared between the odometry task and everything else
// vex::mutex odomMutex;
// double odomX = 0;            // inches
// double odomY = 0;            // inches
// double odomHeadingDeg = 0;   // inertial heading (continuous, not wrapped)
// double odomWheelHeadingDeg = 0; // heading estimated from the tracking wheels only
//
// double prevLeftOdomDeg = 0;
// double prevRightOdomDeg = 0;
// double prevHeadingDeg = 0;
//
// // Heading used by the pose: inertial if it is connected, otherwise wheel-derived
// double currentHeadingDeg() {
//   if (inertialSensor.installed()) {
//     return inertialSensor.rotation(degrees);
//   }
//   return odomWheelHeadingDeg;
// }
//
// // Set the robot pose (x, y in inches, heading in degrees)
// void setPose(double x, double y, double headingDeg) {
//   odomMutex.lock();
//   if (inertialSensor.installed()) {
//     inertialSensor.setRotation(headingDeg, degrees);
//   }
//   odomX = x;
//   odomY = y;
//   odomHeadingDeg = headingDeg;
//   odomWheelHeadingDeg = headingDeg;
//   prevLeftOdomDeg = leftOdom.position(degrees);
//   prevRightOdomDeg = rightOdom.position(degrees);
//   prevHeadingDeg = headingDeg;
//   odomMutex.unlock();
// }
//
// // Thread-safe pose getters
// double getX() {
//   odomMutex.lock();
//   double v = odomX;
//   odomMutex.unlock();
//   return v;
// }
//
// double getY() {
//   odomMutex.lock();
//   double v = odomY;
//   odomMutex.unlock();
//   return v;
// }
//
// double getHeading() {
//   odomMutex.lock();
//   double v = odomHeadingDeg;
//   odomMutex.unlock();
//   return v;
// }
//
// double getWheelHeading() {
//   odomMutex.lock();
//   double v = odomWheelHeadingDeg;
//   odomMutex.unlock();
//   return v;
// }
//
// // Background task: integrates tracking wheel + inertial data into a pose
// int odometryLoop() {
//   while (true) {
//     double leftDeg = leftOdom.position(degrees);
//     double rightDeg = rightOdom.position(degrees);
//
//     double dLeftIn = (leftDeg - prevLeftOdomDeg) * ODOM_IN_PER_DEG;
//     double dRightIn = (rightDeg - prevRightOdomDeg) * ODOM_IN_PER_DEG;
//     prevLeftOdomDeg = leftDeg;
//     prevRightOdomDeg = rightDeg;
//
//     // Heading change from the wheels alone (clockwise positive)
//     double dWheelHeadingDeg =
//       ((dLeftIn - dRightIn) / (2.0 * ODOM_DIST_FROM_CENTER_IN)) * 180.0 / M_PI;
//
//     // Forward travel of the bot center over this step
//     double dForwardIn = (dLeftIn + dRightIn) / 2.0;
//
//     odomMutex.lock();
//     odomWheelHeadingDeg += dWheelHeadingDeg;
//
//     double headingDeg = currentHeadingDeg();
//     // Use the average heading over the step for a better arc approximation
//     double avgHeadingRad = ((prevHeadingDeg + headingDeg) / 2.0) * M_PI / 180.0;
//     prevHeadingDeg = headingDeg;
//
//     odomX += dForwardIn * sin(avgHeadingRad);
//     odomY += dForwardIn * cos(avgHeadingRad);
//     odomHeadingDeg = headingDeg;
//     odomMutex.unlock();
//
//     wait(10, msec);
//   }
//   return 0;
// }

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

// Wrap an angle (degrees) into (-180, 180]
double wrapDeg(double a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// ---- DRIVE-STRAIGHT TUNING ----
// While driving, a separate heading PID keeps the bot on its target heading
// (values are power percent per degree of heading error)
const double HEADING_KP = 1.2;                   // proportional
const double HEADING_KI = 1.5;                   // per degree*second; learns a constant veer (uneven weight)
const double HEADING_KD = 0.06;                  // per degree/second; damps the wobble after a bump or tip
const double HEADING_MAX_CORRECTION = 20.0;      // max total heading correction (power percent)
const double HEADING_MAX_INTEGRAL_POWER = 8.0;   // cap on the integral term (power percent)
const double HEADING_INTEGRAL_WINDOW_DEG = 10.0; // only integrate when heading error is within this
const double HEADING_SETTLE_TOL_DEG = 1.0;       // a move isn't finished until heading is within this

// Optional head start for a constant veer, in power percent at 100% drive power.
// + steers right (clockwise) when driving forward, so use + if the bot veers left. 0 = off.
const double DRIVE_TRIM = 0.0;

// Microtip protection: the acceleration limit tightens as the bot tilts away from level
const double TIP_START_DEG = 1.5;                // below this tilt: normal acceleration (90 or more = off)
const double TIP_FULL_DEG = 5.0;                 // at/above this tilt: tightest acceleration limit
const double TIP_MIN_SLEW_SCALE = 0.35;          // fraction of normal acceleration allowed at full tilt

// Level reference (roll/pitch when the bot sits flat and still), captured in pre_auton
double levelRollDeg = 0;
double levelPitchDeg = 0;

void captureLevel() {
  levelRollDeg = inertialSensor.orientation(roll, degrees);
  levelPitchDeg = inertialSensor.orientation(pitch, degrees);
}

// How far the bot is tilted from level, in any direction (degrees)
double tiltDeg() {
  double rollDelta = wrapDeg(inertialSensor.orientation(roll, degrees) - levelRollDeg);
  double pitchDelta = wrapDeg(inertialSensor.orientation(pitch, degrees) - levelPitchDeg);
  return sqrt(rollDelta * rollDelta + pitchDelta * pitchDelta);
}

// Diagnostics from the most recent drivePID call (call showDriveStats() to display them)
double driveMaxHeadingErrorDeg = 0;
double driveFinalHeadingErrorDeg = 0;
double driveFinalErrorIn = 0;
double drivePeakTiltDeg = 0;
bool driveTimedOut = false;

void showDriveStats() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Max heading err: %.2f deg", driveMaxHeadingErrorDeg);
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("End heading err: %.2f deg", driveFinalHeadingErrorDeg);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("End distance err: %.2f in", driveFinalErrorIn);
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Peak tilt: %.2f deg", drivePeakTiltDeg);
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("Timed out: %s", driveTimedOut ? "yes" : "no");
}

// PID drive: distance comes from odometry, and a separate heading PID keeps the bot on
// targetHeadingDeg (field heading, 0 = +Y, clockwise positive) the whole way.
void drivePIDAtHeading(double inches, double targetHeadingDeg, int timeoutMs = 2000) {
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

  double error = 0;
  double prevPos = 0;
  double integral = 0;
  double derivative = 0;

  const double LOOP_TIME = 20.0; // ms
  const double MAX_DRIVE_POWER = 70.0;
  const double MAX_POWER_CHANGE = 2.5; // percentage points per loop
  const double POWER_FILTER_ALPHA = 0.25;
  int elapsed = 0;
  int settledCount = 0;
  double previousPower = 0;
  double filteredPower = 0;

  // Heading PID state
  double headingIntegral = 0;    // degree * seconds
  double headingDerivative = 0;  // degrees per second (filtered)
  double prevHeading = getHeading();

  // Distance is measured along the target heading
  double startX = getX();
  double startY = getY();
  double targetHeadingRad = targetHeadingDeg * M_PI / 180.0;
  double dirX = sin(targetHeadingRad);
  double dirY = cos(targetHeadingRad);

  // Diagnostics
  driveMaxHeadingErrorDeg = 0;
  driveFinalHeadingErrorDeg = 0;
  driveFinalErrorIn = 0;
  drivePeakTiltDeg = 0;
  driveTimedOut = false;

  while (elapsed < timeoutMs) {
    // Signed distance traveled along the target heading (inches) from odometry
    double traveledIn = (getX() - startX) * dirX + (getY() - startY) * dirY;
    double avgPos = traveledIn * motorDegPerInch;

    error = targetMotorDeg - avgPos;

    // Heading error, shortest direction (positive = bot is left of the target heading)
    double heading = getHeading();
    double headingError = wrapDeg(targetHeadingDeg - heading);
    if (fabs(headingError) > driveMaxHeadingErrorDeg) driveMaxHeadingErrorDeg = fabs(headingError);

    // Settling check: distance within 2 degrees AND heading within tolerance,
    // for 5 consecutive loops (100ms)
    if (fabs(error) < 2.0 && fabs(headingError) < HEADING_SETTLE_TOL_DEG) {
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

    // Microtip protection: tighten the acceleration limit while the bot is tilted
    double tilt = tiltDeg();
    if (tilt > drivePeakTiltDeg) drivePeakTiltDeg = tilt;
    double slewScale = 1.0;
    if (tilt > TIP_START_DEG) {
      double t = fmin(1.0, (tilt - TIP_START_DEG) / (TIP_FULL_DEG - TIP_START_DEG));
      slewScale = 1.0 - (1.0 - TIP_MIN_SLEW_SCALE) * t;
    }
    double maxPowerChange = MAX_POWER_CHANGE * slewScale;

    double powerChange = power - previousPower;
    if (powerChange > maxPowerChange) {
      power = previousPower + maxPowerChange;
    } else if (powerChange < -maxPowerChange) {
      power = previousPower - maxPowerChange;
    }
    previousPower = power;

    // Heading PID (runs the whole move, including at low power and at the start/end)
    double headingRate = (heading - prevHeading) / (LOOP_TIME / 1000.0); // degrees per second
    prevHeading = heading;
    headingDerivative += 0.3 * (headingRate - headingDerivative);

    if (fabs(headingError) < HEADING_INTEGRAL_WINDOW_DEG) {
      headingIntegral += headingError * (LOOP_TIME / 1000.0);
    }
    double maxHeadingIntegral = HEADING_MAX_INTEGRAL_POWER / HEADING_KP;
    if (headingIntegral > maxHeadingIntegral) headingIntegral = maxHeadingIntegral;
    if (headingIntegral < -maxHeadingIntegral) headingIntegral = -maxHeadingIntegral;

    double headingCorrection = (HEADING_KP * headingError) +
                               (HEADING_KI * headingIntegral) -
                               (HEADING_KD * headingDerivative);
    if (headingCorrection > HEADING_MAX_CORRECTION) headingCorrection = HEADING_MAX_CORRECTION;
    if (headingCorrection < -HEADING_MAX_CORRECTION) headingCorrection = -HEADING_MAX_CORRECTION;
    headingCorrection += DRIVE_TRIM;

    // Fade the correction out as power drops so it can't spin the bot at the end
    headingCorrection *= fmin(1.0, fabs(power) / 20.0);

    double leftPower = power + headingCorrection;
    double rightPower = power - headingCorrection;
    if (leftPower > 100) leftPower = 100;
    if (leftPower < -100) leftPower = -100;
    if (rightPower > 100) rightPower = 100;
    if (rightPower < -100) rightPower = -100;

    leftDrive.spin(fwd, leftPower, pct);
    rightDrive.spin(fwd, rightPower, pct);

    wait(LOOP_TIME, msec);
    elapsed += LOOP_TIME;
  }

  // Record how the move ended (see showDriveStats)
  driveTimedOut = (settledCount < 5);
  driveFinalHeadingErrorDeg = wrapDeg(targetHeadingDeg - getHeading());
  driveFinalErrorIn = inches - ((getX() - startX) * dirX + (getY() - startY) * dirY);

  // Passive electromagnetic braking mode (protects motors from thermal strain)
  leftDrive.stop(brake);
  rightDrive.stop(brake);
}

// Drive straight along the heading the bot is currently facing
void drivePID(double inches, int timeoutMs = 2000) {
  drivePIDAtHeading(inches, getHeading(), timeoutMs);
}

// Turn in place to an absolute field heading (degrees, 0 = +Y, clockwise positive).
// Always takes the shortest direction.
void turnToHeading(double targetHeadingDeg, int timeoutMs = 1500) {
  // Tune these (power percent per degree of error)
  double kP = 0.8;
  double kI = 0.01;
  double kD = 0.06;

  const double MAX_TURN_POWER = 70.0;
  const double MIN_TURN_POWER = 4.0;    // overcomes static friction when close
  const double SETTLE_TOL_DEG = 1.0;
  const int SETTLE_LOOPS = 5;           // 50ms inside tolerance
  const int LOOP_TIME = 10;             // ms

  double integral = 0;
  double derivative = 0;
  double prevError = wrapDeg(targetHeadingDeg - getHeading());
  int elapsed = 0;
  int settledCount = 0;

  while (elapsed < timeoutMs) {
    double error = wrapDeg(targetHeadingDeg - getHeading());

    if (fabs(error) < SETTLE_TOL_DEG) {
      settledCount++;
    } else {
      settledCount = 0;
    }
    if (settledCount >= SETTLE_LOOPS) break;

    // Integral only near the target, reset when crossing it
    if (fabs(error) < 10.0) {
      integral += error;
    } else {
      integral = 0;
    }
    if ((error > 0) != (prevError > 0)) integral = 0;
    if (integral > 1000) integral = 1000;
    if (integral < -1000) integral = -1000;

    // Filtered derivative in degrees per second
    double rawDerivative = (error - prevError) / (LOOP_TIME / 1000.0);
    derivative += 0.3 * (rawDerivative - derivative);
    prevError = error;

    double power = (kP * error) + (kI * integral) + (kD * derivative);

    if (power > MAX_TURN_POWER) power = MAX_TURN_POWER;
    if (power < -MAX_TURN_POWER) power = -MAX_TURN_POWER;
    if (fabs(error) > 1.5 && fabs(power) < MIN_TURN_POWER) {
      power = (power >= 0) ? MIN_TURN_POWER : -MIN_TURN_POWER;
    }

    // Positive error = need to turn clockwise: left forward, right backward
    leftDrive.spin(fwd, power, pct);
    rightDrive.spin(fwd, -power, pct);

    wait(LOOP_TIME, msec);
    elapsed += LOOP_TIME;
  }

  leftDrive.stop(brake);
  rightDrive.stop(brake);
  wait(50, msec);
}

// Turn in place so the robot faces the field point (x, y) in inches.
// reverse = true points the back of the robot at the point instead.
void turnToPoint(double x, double y, bool reverse = false, int timeoutMs = 1500) {
  double dx = x - getX();
  double dy = y - getY();

  // Too close to the point for a meaningful direction
  if (sqrt(dx * dx + dy * dy) < 0.5) return;

  // atan2(dx, dy): 0 = +Y, clockwise positive (matches the pose heading)
  double targetDeg = atan2(dx, dy) * 180.0 / M_PI;
  if (reverse) targetDeg += 180.0;

  turnToHeading(targetDeg, timeoutMs);
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

  // Remember what "level" reads so tilt can be measured during moves
  captureLevel();

  // Start the pose at the origin facing +Y
  setPose(0, 0, 0);
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Tuning
  // drivePID(12);
  // turnToHeading(90);
  // turnToPoint(24, 24);

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