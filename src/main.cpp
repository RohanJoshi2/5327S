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
const double ODOM_OFFSET_FROM_CENTER_IN = 0.25; // MEASURE. Signed: + = wheel is RIGHT of bot center, - = LEFT
const bool ODOM_REVERSED = true;               // flip if the sensor reads negative when driving forward

// Two tracking wheels (DISABLED)
// const double ODOM_DIST_FROM_CENTER_IN = 5.0; // distance from bot center to each tracking wheel (inches)
// const bool LEFT_ODOM_REVERSED = false;       // flip if the sensor reads negative when driving forward
// const bool RIGHT_ODOM_REVERSED = false;      // flip if the sensor reads negative when driving forward

// Misc Motors
motor toggleMotor = motor(PORT18, true); // 5.5W
digital_out clawPneumatic = digital_out(Brain.ThreeWirePort.H);
bool clawPneumaticOpen = false;

// Sensors
inertial inertialSensor = inertial(PORT17);
rotation odomWheel = rotation(PORT7, ODOM_REVERSED); // single tracking wheel (ACTIVE)

// Two tracking wheels (DISABLED)
// rotation leftOdom = rotation(PORT12, LEFT_ODOM_REVERSED);
// rotation rightOdom = rotation(PORT13, RIGHT_ODOM_REVERSED);

// DR4B motors
motor DR4BLeft = motor(PORT9, false);
motor DR4BRight = motor(PORT12, true);

// Left side
motor leftMotor1 = motor(PORT4, ratio6_1, false);   // 5.5W
motor leftMotor2 = motor(PORT5, ratio6_1, true);   // 11W
motor leftMotor3 = motor(PORT6, ratio6_1, true);   // 11W

// Right side
motor rightMotor1 = motor(PORT1, ratio6_1, true);   // 5.5W
motor rightMotor2 = motor(PORT2, ratio6_1, false);   // 11W
motor rightMotor3 = motor(PORT3, ratio6_1, false);   // 11W

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
// The latest 24 in command traveled 24.75 in; raise scale so odometry stops sooner.
const double ODOM_DISTANCE_SCALE = 1.062239884;
const double ODOM_IN_PER_DEG = (ODOM_WHEEL_CIRC_IN / 360.0) * ODOM_DISTANCE_SCALE;

// Pose shared between the odometry task and everything else
mutex odomMutex;
double odomX = 0;            // inches
double odomY = 0;            // inches
double odomHeadingDeg = 0;   // inertial rotation (continuous, not wrapped)

double prevOdomDeg = 0;
double prevHeadingDeg = 0;

// Heading the robot is SUPPOSED to be facing. turnClockwise() adds to it and
// drivePID() holds it, so a turn that lands 1 deg off is not carried forward.
double commandedHeadingDeg = 0;

// Set the robot pose (x, y in inches, heading in degrees)
void setPose(double x, double y, double headingDeg) {
  odomMutex.lock();
  commandedHeadingDeg = headingDeg;
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

// If the robot was knocked far from the commanded heading (collision, being
// pushed), trust where it actually is instead of fighting back to the old heading.
const double HEADING_RESYNC_DEG = 15.0;
void syncCommandedHeading() {
  if (fabs(commandedHeadingDeg - getHeading()) > HEADING_RESYNC_DEG) {
    commandedHeadingDeg = getHeading();
  }
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
    DR4BMotors.spin(forward, 40, percent);
  } else if (down && !up) {
    DR4BMotors.spin(reverse, 40, percent);
  } else {
    DR4BMotors.stop(hold);
  }
}

void spinToggle(bool active) {
  if (active) {
    toggleMotor.spin(forward, 70, percent);
  } else {
    toggleMotor.stop(coast);
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

void spinToggleFor(bool active, int durationMs) {
  if (active) {
    toggleMotor.spin(forward, 100, percent);
  } else {
    toggleMotor.stop(coast);
  }
  wait(durationMs, msec);
  toggleMotor.stop(coast);
}

// Arcade drive function
void arcadeDrive() {
  int throttle = Controller1.Axis3.position();
  int turn = Controller1.Axis1.position();

  if (throttle > -5 && throttle < 5) throttle = 0;
  if (turn > -5 && turn < 5) turn = 0;

  throttle = static_cast<int>(0.7 * throttle);
  turn = static_cast<int>(0.7 * turn);

  // Positive turn input should turn clockwise, matching turnClockwise().
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

void turnClockwise(int deg) {
  if (deg == 0) return;

  const int LOOP_TIME_MS = 10;
  const double LOOP_TIME_SEC = LOOP_TIME_MS / 1000.0;
  const int TIMEOUT_MS = 1500 + static_cast<int>(fabs(static_cast<double>(deg)) * 10.0);
  const double KP = 0.8;
  const double KI = 0.01;
  const double KD = 0.06;
  const double MAX_TURN_POWER = 70.0;
  const double MIN_TURN_POWER = 4.0;
  const double SETTLE_TOL_DEG = 1.0;
  const int SETTLE_LOOPS = 5;

  // Build on the commanded heading (not the raw current one) so turn errors
  // do not accumulate across a multi-turn auton.
  syncCommandedHeading();
  const double targetHeading = commandedHeadingDeg + deg;
  commandedHeadingDeg = targetHeading;
  double previousHeading = inertialSensor.rotation(degrees);
  double integral = 0;
  double filteredRate = 0;
  int elapsedMs = 0;
  int settledCount = 0;

  while (elapsedMs < TIMEOUT_MS) {
    double heading = inertialSensor.rotation(degrees);
    double error = targetHeading - heading;

    if (fabs(error) < SETTLE_TOL_DEG) {
      settledCount++;
    } else {
      settledCount = 0;
    }
    if (settledCount >= SETTLE_LOOPS) break;

    if (fabs(error) < 10.0) {
      integral += error * LOOP_TIME_SEC;
    } else {
      integral = 0;
    }
    if (integral > 100.0) integral = 100.0;
    if (integral < -100.0) integral = -100.0;

    double headingRate = (heading - previousHeading) / LOOP_TIME_SEC;
    previousHeading = heading;
    filteredRate += 0.3 * (headingRate - filteredRate);

    double power = KP * error + KI * integral - KD * filteredRate;
    if (power > MAX_TURN_POWER) power = MAX_TURN_POWER;
    if (power < -MAX_TURN_POWER) power = -MAX_TURN_POWER;
    if (fabs(error) > 1.5 && fabs(power) < MIN_TURN_POWER) {
      power = error > 0 ? MIN_TURN_POWER : -MIN_TURN_POWER;
    }

    leftDrive.spin(fwd, power, pct);
    rightDrive.spin(fwd, -power, pct);

    wait(LOOP_TIME_MS, msec);
    elapsedMs += LOOP_TIME_MS;
  }

  leftDrive.stop(brake);
  rightDrive.stop(brake);
  wait(50, msec);
}

// Wrap an angle (degrees) into (-180, 180]
double wrapDeg(double a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// ---- DRIVE-STRAIGHT TUNING ----
// Drive straight while holding the current heading. This keeps the drive system
// separate from the turning system instead of targeting a specific absolute heading.
const double DRIVE_KP = 0.12;
const double DRIVE_KI = 0.02;
const double DRIVE_KD = 0.003;
const double DRIVE_MAX_POWER = 70.0;
const double DRIVE_MAX_POWER_CHANGE = 1.0;
const double DRIVE_POWER_FILTER_ALPHA = 0.25;

// Heading hold (runs the whole move). Positive heading error = robot must turn
// clockwise = left side gets more power, right side less.
const double HEADING_KP = 0.3;
const double HEADING_KI = 0.04;
const double HEADING_KD = 0.03;
const double HEADING_MAX_CORRECTION = 6.0;
const double HEADING_MAX_INTEGRAL_POWER = 1.2;
const double HEADING_INTEGRAL_WINDOW_DEG = 8.0;
const double HEADING_FADE_POWER = 7.0;  // correction fades below this drive power

const double TIP_RATE_THRESHOLD_DPS = 8.0;
const double TIP_SIDE_BOOST_POWER = 0.8;

const double TIP_START_DEG = 1.5;
const double TIP_FULL_DEG = 5.0;
const double TIP_MIN_SLEW_SCALE = 0.35;

double levelRollDeg = 0;
double levelPitchDeg = 0;

void captureLevel() {
  levelRollDeg = inertialSensor.orientation(roll, degrees);
  levelPitchDeg = inertialSensor.orientation(pitch, degrees);
}

double tiltDeg() {
  double rollDelta = wrapDeg(inertialSensor.orientation(roll, degrees) - levelRollDeg);
  double pitchDelta = wrapDeg(inertialSensor.orientation(pitch, degrees) - levelPitchDeg);
  return sqrt(rollDelta * rollDelta + pitchDelta * pitchDelta);
}

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

// Drive straight along the robot's current heading, without commanding a target heading.
void drivePID(double inches, int timeoutMs = 3500) {
  if (inches == 0) return;

  const double wheelCircumference = 2.75 * M_PI;
  const double motorDegPerInch = 1.09 * (360.0 / wheelCircumference);
  double targetMotorDeg = inches * motorDegPerInch;

  double error = 0;
  double prevPos = 0;
  double integral = 0;
  double derivative = 0;

  const double LOOP_TIME = 20.0;
  int elapsed = 0;
  int settledCount = 0;
  double previousPower = 0;
  double filteredPower = 0;
  double headingIntegral = 0;

  double startX = getX();
  double startY = getY();
  double targetHeadingDeg = getHeading();
  double targetHeadingRad = targetHeadingDeg * M_PI / 180.0;
  double dirX = sin(targetHeadingRad);
  double dirY = cos(targetHeadingRad);

  driveMaxHeadingErrorDeg = 0;
  driveFinalHeadingErrorDeg = 0;
  driveFinalErrorIn = 0;
  drivePeakTiltDeg = 0;
  driveTimedOut = false;
  bool reachedTarget = false;

  while (elapsed < timeoutMs) {
    double traveledIn = (getX() - startX) * dirX + (getY() - startY) * dirY;
    double avgPos = traveledIn * motorDegPerInch;

    error = targetMotorDeg - avgPos;

    double heading = getHeading();
    double headingError = wrapDeg(targetHeadingDeg - heading);
    if (fabs(headingError) > driveMaxHeadingErrorDeg) driveMaxHeadingErrorDeg = fabs(headingError);

    // Stop at the target rather than reversing to correct an overshoot.
    if ((inches > 0 && error <= 0) || (inches < 0 && error >= 0)) {
      reachedTarget = true;
      break;
    }

    if (fabs(error) < 2.0) {
      settledCount++;
    } else {
      settledCount = 0;
    }
    if (settledCount >= 5) {
      reachedTarget = true;
      break;
    }

    if (fabs(error) < 40.0) {
      integral += error;
    } else {
      integral = 0;
    }
    if (integral > 3000) integral = 3000;
    if (integral < -3000) integral = -3000;

    double measuredVelocity = (avgPos - prevPos) / (LOOP_TIME / 1000.0);
    derivative += 0.25 * (measuredVelocity - derivative);
    prevPos = avgPos;

    double power = (DRIVE_KP * error) + (DRIVE_KI * integral) - (DRIVE_KD * derivative);
    if (power > DRIVE_MAX_POWER) power = DRIVE_MAX_POWER;
    if (power < -DRIVE_MAX_POWER) power = -DRIVE_MAX_POWER;
    if (fabs(power) < 3.0) power = 0;

    filteredPower += DRIVE_POWER_FILTER_ALPHA * (power - filteredPower);
    power = filteredPower;

    double tilt = tiltDeg();
    if (tilt > drivePeakTiltDeg) drivePeakTiltDeg = tilt;
    double slewScale = 1.0;
    if (tilt > TIP_START_DEG) {
      double t = fmin(1.0, (tilt - TIP_START_DEG) / (TIP_FULL_DEG - TIP_START_DEG));
      slewScale = 1.0 - (1.0 - TIP_MIN_SLEW_SCALE) * t;
    }
    double maxPowerChange = DRIVE_MAX_POWER_CHANGE * slewScale;

    double powerChange = power - previousPower;
    if (powerChange > maxPowerChange) {
      power = previousPower + maxPowerChange;
    } else if (powerChange < -maxPowerChange) {
      power = previousPower - maxPowerChange;
    }
    if (inches > 0 && power < 0) power = 0;
    if (inches < 0 && power > 0) power = 0;
    previousPower = power;

    double leftPower = power;
    double rightPower = power;

    // Z-axis yaw rate: positive = turning clockwise.
    double yawRateDps = inertialSensor.installed() ? inertialSensor.gyroRate(zaxis, dps) : 0.0;

    // IMU holds the heading captured at the start; odometry is used for distance only.
    double aimError = wrapDeg(targetHeadingDeg - heading);
    if (fabs(aimError) < HEADING_INTEGRAL_WINDOW_DEG) {
      headingIntegral += aimError * (LOOP_TIME / 1000.0);
    } else {
      headingIntegral = 0;
    }
    double maxIntegral = HEADING_KI > 0 ? HEADING_MAX_INTEGRAL_POWER / HEADING_KI : 0;
    if (headingIntegral > maxIntegral) headingIntegral = maxIntegral;
    if (headingIntegral < -maxIntegral) headingIntegral = -maxIntegral;

    double correction = (HEADING_KP * aimError) + (HEADING_KI * headingIntegral) - (HEADING_KD * yawRateDps);
    if (correction > HEADING_MAX_CORRECTION) correction = HEADING_MAX_CORRECTION;
    if (correction < -HEADING_MAX_CORRECTION) correction = -HEADING_MAX_CORRECTION;
    correction *= fmin(1.0, fabs(power) / HEADING_FADE_POWER);
    leftPower += correction;
    rightPower -= correction;

    if (inertialSensor.installed()) {
      // Fast-reaction boost on top of the PID: Z rate clockwise (positive) boosts
      // the RIGHT side, counterclockwise (negative) boosts the LEFT side. Reverse
      // travel naturally swaps sides because power changes sign.
      if (fabs(yawRateDps) > TIP_RATE_THRESHOLD_DPS) {
        double boost = fmin(TIP_SIDE_BOOST_POWER, fabs(power) * 0.25);
        if (yawRateDps * power > 0) {
          rightPower += (power >= 0 ? 1.0 : -1.0) * boost;
        } else {
          leftPower += (power >= 0 ? 1.0 : -1.0) * boost;
        }
      }
    }
    if (leftPower > DRIVE_MAX_POWER) leftPower = DRIVE_MAX_POWER;
    if (leftPower < -DRIVE_MAX_POWER) leftPower = -DRIVE_MAX_POWER;
    if (rightPower > DRIVE_MAX_POWER) rightPower = DRIVE_MAX_POWER;
    if (rightPower < -DRIVE_MAX_POWER) rightPower = -DRIVE_MAX_POWER;

    leftDrive.spin(fwd, leftPower, pct);
    rightDrive.spin(fwd, rightPower, pct);

    wait(LOOP_TIME, msec);
    elapsed += LOOP_TIME;
  }

  driveTimedOut = !reachedTarget;
  driveFinalHeadingErrorDeg = wrapDeg(targetHeadingDeg - getHeading());
  driveFinalErrorIn = inches - ((getX() - startX) * dirX + (getY() - startY) * dirY);

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

  toggleClaw(); // Start with the claw closed

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
  drivePID(24);

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
    spinToggle(Controller1.ButtonL1.pressing());
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