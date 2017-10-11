package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;

public class UltrasonicLocalizer extends Thread {
  // enumeration
  public enum LocalizationType {
    FALLING_EDGE, RISING_EDGE
  }

  private LocalizationType localizationType; // chosen type, as passed in from the main program

  // motors and stuff
  private NXTRegulatedMotor leftMotor;
  private NXTRegulatedMotor rightMotor;
  private EV3MediumRegulatedMotor sensorMotor;
  private Odometer odometer;
  private Navigation navigation;
  private UltrasonicPoller usPoller;

  // orientation position variables
  private double thetaA;
  private double thetaB;
  private double deltaTheta;

  // some constants (as per the main class; refer for description)
  private static int rotateSpeed = LocalizationLab.ROTATE_SPEED;
  private static int acceleration = LocalizationLab.MOTOR_ACCELERATION;
  private static int defaultAcceleration = LocalizationLab.DEFAULT_ACCELERATION;
  private static int threshold = LocalizationLab.THRESHOLD;
  private static int noiseMargin = LocalizationLab.NOISE_MARGIN;

  // class-specific constants
  private static final int FALLING_THETA_ERROR_1 = -20; // Error to correct the given 45° and 255°
                                                        // values by
  private static final int RISING_THETA_ERROR_1 = -30;
  private static final int RISING_THETA_ERROR_2 = -20;


  // constructor
  public UltrasonicLocalizer(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor,
      EV3MediumRegulatedMotor sensorMotor, Odometer odometer, Navigation aNavigation,
      UltrasonicPoller usPoller) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.sensorMotor = sensorMotor;
    this.odometer = odometer;
    this.navigation = aNavigation;
    this.usPoller = usPoller;
  }

  /**
   * Main method: performs ultrasonic localization
   * 
   * @param aLocalizationType The type (rising/falling edge) of localization wanted
   */
  public void localize(LocalizationType aLocalizationType) {

    sensorMotor.stop(); // prevent the sensor motor from moving

    // set the chosen localization type
    localizationType = aLocalizationType;

    // set the appropriate acceleration
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);

    if (localizationType == LocalizationType.RISING_EDGE) {
      risingEdge();
    } else if (localizationType == LocalizationType.FALLING_EDGE) {
      fallingEdge();
    } else { // default or invalid option; should not happen.
      return;
    }

    // compute deltaTheta, store it in the appropriate variable
    computeDeltaTheta();

    // adjust the odometer's theta value
    double currentTheta = odometer.getTheta();
    odometer.setTheta(currentTheta + deltaTheta);

    // turn the robot to the *hopefully* appropriate 0° heading
    navigation.turnTo(0);
  }

  /**
   * Executes the rising edge way of identifying walls
   */
  public void risingEdge() {

    // 1. getting thetaA: angle of horizontal wall is seen (where thetaA < thetaB)

    // start rotating CCW
    rotateCCW();

    // check: what if we started not facing a wall?
    // wait until the US says that we're not anymore
    while (usPoller.getDistance() > threshold - noiseMargin);

    // continue rotating cuz why not
    rotateCCW();
    // sleep to avoid detecting a re-detection.
    try {
      Thread.sleep(2500);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }

    // more wait while it's still seeing a wall
    while (usPoller.getDistance() < threshold);

    // a rising edge has been detected!!!
    // stop the motors
    Sound.playNote(Sound.PIANO, 440, 200);
    stopMotors();

    // get the angle from the odometer; store as thetaA
    thetaA = odometer.getTheta();

    // 2. getting thetaB: angle of vertical wall (where thetaA < thetaB)

    rotateCW();
    while (usPoller.getDistance() > threshold - noiseMargin);
    // rotate CW
    rotateCW();
    // more sleeping to avoid re-detecting the same wall/detecting a false non-wall
    try {
      Thread.sleep(2500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // more waiting
    while (usPoller.getDistance() < threshold);

    // another rising edge has been detected!!!
    // stop the motors
    Sound.playNote(Sound.PIANO, 440, 200);
    stopMotors();

    // get the angle from the odometer; store as thetaB
    thetaB = odometer.getTheta();
  }

  /**
   * Executes the rising edge way of identifying walls
   */
  public void fallingEdge() {

    // 1. getting thetaA: angle of horizontal wall is seen (where thetaA < thetaB)

    // start rotating CW
    rotateCW();

    // check: what if we started already facing a wall?
    // wait until the US says that we're not anymore
    while (usPoller.getDistance() < threshold + noiseMargin);

    // continue rotating cuz why not
    rotateCW();
    // sleep to avoid re-detection
    try {
      Thread.sleep(2500);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }

    // more wait while it's still seeing a wall
    while (usPoller.getDistance() > threshold);

    // a falling edge has been detected!!!
    // stop the motors
    Sound.playNote(Sound.PIANO, 440, 200);
    stopMotors();

    // get the angle from the odometer; store as thetaA
    thetaA = odometer.getTheta();

    // 2. getting thetaB: angle of vertical wall (where thetaA < thetaB)

    // rotate CCW
    rotateCCW();
    while (usPoller.getDistance() < threshold + noiseMargin);
    rotateCCW();
    // more sleeping to avoid re-detecting the same wall/detecting a false non-wall
    try {
      Thread.sleep(2500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // more waiting
    while (usPoller.getDistance() > threshold);

    // another falling edge has been detected!!!
    // stop the motors
    Sound.playNote(Sound.PIANO, 440, 200);
    stopMotors();

    // get the angle from the odometer; store as thetaB
    thetaB = odometer.getTheta();
  }

  /**
   * Computes the change of angle to be added to the odometer after rising/falling edge has been
   * executed
   * 
   * @return deltaTheta
   */
  private double computeDeltaTheta() {

    if (localizationType == LocalizationType.FALLING_EDGE) {
      // case 1
      if (thetaA > thetaB) {
        deltaTheta = 45 - (thetaA + thetaB) / 2.0;
      }

      // case 2
      else {
        deltaTheta = (255 + FALLING_THETA_ERROR_1) - ((thetaA + thetaB) / 2.0);
      }

    } else {
      // case 1
      if (thetaA > thetaB) {
        deltaTheta = 45 - (thetaA + thetaB) / 2.0;
      }

      // case 2
      else if (thetaA < 180) {
        deltaTheta = (255 + RISING_THETA_ERROR_1) - ((thetaA + thetaB) / 2.0);
      }

      // case 3
      else {
        deltaTheta = (255 + RISING_THETA_ERROR_2) - ((thetaA + thetaB) / 2.0);
      }
    }
    return deltaTheta;
  }

  /**
   * Rotates CW with rotate speed.
   */
  private void rotateCW() {
    leftMotor.setSpeed(rotateSpeed);
    rightMotor.setSpeed(rotateSpeed);
    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * Rotates CCW with rotate speed.
   */
  private void rotateCCW() {
    leftMotor.setSpeed(rotateSpeed);
    rightMotor.setSpeed(rotateSpeed);
    leftMotor.backward();
    rightMotor.forward();
  }

  /**
   * Stops the motors with the default acceleration
   */
  private void stopMotors() {
    leftMotor.setAcceleration(defaultAcceleration);
    rightMotor.setAcceleration(defaultAcceleration);
    leftMotor.stop(true);
    rightMotor.stop();
  }

}
