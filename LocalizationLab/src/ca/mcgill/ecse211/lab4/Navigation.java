package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Navigation extends Thread {

  // motors and stuff
  private NXTRegulatedMotor leftMotor;
  private NXTRegulatedMotor rightMotor;
  private Odometer odometer;

  // heading
  public static double heading; // computed heading to destination from current position (when
                                // needed)

  // some constants (as per the main class; refer for description)
  private double tileLength = LocalizationLab.TILE_LENGTH;
  private int forwardSpeed = LocalizationLab.MOTOR_HIGH;
  private double wheelRadius = LocalizationLab.WHEEL_RADIUS;
  private double track = LocalizationLab.TRACK;
  private int rotateSpeed = LocalizationLab.ROTATE_SPEED;
  private int acceleration = LocalizationLab.MOTOR_ACCELERATION;


  // constructor
  public Navigation(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor,
      EV3MediumRegulatedMotor sensorMotor, Odometer odometer) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.odometer = odometer;
  }

  // in order to start the thread
  public void run() {
    // we do not need to execute navigation in this lab
  }

  public void travelTo(double pointX, double pointY) {
    double minAngle, travelDistance, distanceX, distanceY, deltaX, deltaY; // angleDifference;

    // set an appropriate acceleration to reduce odometer error and increase smoothness
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);

    // convert point coordinates to actual distance *in cm*
    distanceX = pointX * tileLength;
    distanceY = pointY * tileLength;

    // calculate distance to travel from current position in each direction
    deltaX = distanceX - odometer.getX();
    deltaY = distanceY - odometer.getY();

    // calculate pythagorean distance to travel
    travelDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

    // turn towards the required angle
    minAngle = computeHeading(distanceX, distanceY); // compute the required heading to turn to
    turnTo(minAngle);

    // travel to destination
    leftMotor.setSpeed(forwardSpeed);
    rightMotor.setSpeed(forwardSpeed);
    leftMotor.rotate(convertDistance(wheelRadius, travelDistance), true);
    rightMotor.rotate(convertDistance(wheelRadius, travelDistance), false);
  }

  /**
   * method to turn the robot to the desired heading with *hopefully* the minimal angle Positive Y
   * axis is 0°; angle going CW
   * 
   * @param theta
   */
  public void turnTo(double theta) {
    // stored previous coordinates before a turn is made
    // used to correct any errors on odometer during a turn
    double previousX;
    double previousY;

    previousX = odometer.getX();
    previousY = odometer.getY();

    leftMotor.setSpeed(rotateSpeed);
    rightMotor.setSpeed(rotateSpeed);

    // difference between current angle and the desired heading
    double angleDifference = theta - odometer.getTheta();

    // turning to the right
    // Case 1: Positive and [0, 180[
    if ((0 <= angleDifference && angleDifference <= 180)) {
      leftMotor.rotate(convertAngle(wheelRadius, track, angleDifference), true);
      rightMotor.rotate(-convertAngle(wheelRadius, track, angleDifference), false);
      Sound.playNote(Sound.PIANO, 440, 200);
    }
    // Case 2: Negative and ]-360, -180]
    else if (angleDifference < -180) {
      leftMotor.rotate(convertAngle(wheelRadius, track, angleDifference + 360), true);
      rightMotor.rotate(-convertAngle(wheelRadius, track, angleDifference + 360), false);
      Sound.playNote(Sound.PIANO, 500, 200);
    }

    // turning to the left
    // Case 3: Positive and ]180, 360[
    else if (angleDifference > 180) {
      leftMotor.rotate(-convertAngle(wheelRadius, track, 360 - angleDifference), true);
      rightMotor.rotate(convertAngle(wheelRadius, track, 360 - angleDifference), false);
      Sound.playNote(Sound.PIANO, 880, 200);
    }
    // Case 4: Negative and ]-180, 0[
    else {
      leftMotor.rotate(convertAngle(wheelRadius, track, angleDifference), true);
      rightMotor.rotate(-convertAngle(wheelRadius, track, angleDifference), false);
    }

    odometer.setX(previousX);
    odometer.setY(previousY);
  }


  // This method returns true if another thread has called travelTo() or turnTo()
  // and the method has yet to return; false otherwise.


  /**
   * This method returns true if another thread has called travelTo() or turnTo() and the method has
   * yet to return; false otherwise. Simply put, checks if either wheel is moving.
   * 
   * @param
   * @return isNavigating: boolean
   */
  boolean isNavigating() {
    boolean isNavigating;
    isNavigating = leftMotor.isMoving() || rightMotor.isMoving();
    return isNavigating;
  }

  /**
   * Converts a given distance into number of wheel rotations for a wheel with a specific radius
   * 
   * @param radius
   * @param distance
   * @return int
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }


  /**
   * Converts a given angle into number of wheel rotations for a wheel with a specific radius
   * 
   * @param radius
   * @param width
   * @param angle
   * @return int
   * 
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  // takes in distance (coordinates in cm, not points)
  /**
   * calculates the required heading from current position to required position takes in coordinates
   * from odometer
   * 
   * @param destinationX
   * @param destinationY
   * @return double
   * 
   */
  private double computeHeading(double destinationX, double destinationY) {
    // calculate distance to travel from current position in each direction
    double deltaX, deltaY, angle;

    deltaX = destinationX - odometer.getX();
    deltaY = destinationY - odometer.getY();

    // takes in x value first since 0° is at the y axis
    angle = Math.atan2(deltaX, deltaY); // gets the angle in the correct quadrant [-180, 180]
    if (angle < 0) { // so that the range of theta stays [0, 2pi[ or [0°, 359°]
      angle += Math.PI * 2;
    }

    // convert into degrees
    angle = Math.toDegrees(angle);
    heading = angle;
    return angle;
  }
}
