package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.SensorModes;

public class LightLocalizer extends Thread {

  // motors and stuff
  private NXTRegulatedMotor leftMotor;
  private NXTRegulatedMotor rightMotor;
  private Odometer odometer;
  private Navigation navigation;

  // used to store light sensor information
  private SensorModes myLight;
  private float[] sampleLight;

  // current light sensor value
  private float lightSensorValue;

  // boolean to prevent a same line to be read twice
  private boolean readLine;

  // array to store the four angles read by the light sensor.
  private double[] thetaArray = new double[4];
  
  // position information
  private double thetaY;
  private double thetaX;

  // some constants (as per the main class; refer for description)
  private int forwardSpeed = LocalizationLab.MOTOR_HIGH;
  private int rotateSpeed = LocalizationLab.ROTATE_SPEED;
  private int acceleration = LocalizationLab.MOTOR_ACCELERATION;
  private double wheelRadius = LocalizationLab.WHEEL_RADIUS;
  private double track = LocalizationLab.TRACK;
  // TODO: added(?)
  private double sensorDistance = LocalizationLab.SENSOR_DISTANCE;
  private double blackLine = LocalizationLab.BLACK_LINE;
  private int defaultAcceleration = LocalizationLab.DEFAULT_ACCELERATION;

  // constructor
  public LightLocalizer(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor,
      Odometer odometer, Navigation aNavigation, SensorModes mylight, float[] sampleLight) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.odometer = odometer;
    this.navigation = aNavigation;
    this.myLight = mylight;
    this.sampleLight = sampleLight;
  }

  /**
   * Main method: correct the x, y position of the robot using the light sensor
   */
  public void localize(boolean fixPosition) {
    
    //turn the robot CCW
    turnCCW();
    
    //while turning, store the four theta values read by the light sensor
    int counter = 0; //counter for knowing how many black lines are read
    readLine = false;
    while (counter < 4 && (leftMotor.isMoving() || rightMotor.isMoving())) {

      myLight.fetchSample(sampleLight, 0); // store a data sample in array sampleLight starting at
                                           // index 0
      lightSensorValue = sampleLight[0]; // store that value in a variable

      // if robot is on the same black line for the first time
      if (lightSensorValue >= blackLine && !readLine) {

        // passed a line: store value in array and increment the line counter
        thetaArray[counter] = odometer.getTheta();
        counter++;
        
        readLine = true; // finished reading the line.
        Sound.playNote(Sound.PIANO, 880, 200);
        
      } else { // either actually not on a black line OR one one but already finished reading it
        readLine = false;
      }
    }

    //stop the robot when it has done a full turn
    while (Math.abs(odometer.getTheta() - 0) > 1); // wait until does full turn
    // stop motors
    stopMotors();

    //wait a bit before computing position, why not
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // compute & store correct position in odometer
    computePosition();
    computeAngle(); //TODO
    
    //wait more. slow and steady wins the race.
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // travel to *hopefully* (0,0)
    navigation.travelTo(0, 0);
    navigation.turnTo(0);

  }
  
  /**
   * Rotates CCW with rotate speed.
   */
  private void turnCCW() {
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);
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


  /**
   * Computes the correct x, y position of the robot and stores them in the odometer
   */
  private void computePosition() {

    double x;
    double y;

    // compute thetaY and theta X
    thetaY = Math.abs(thetaArray[2] - thetaArray[0]); //difference btwn the first and third line
    thetaX = Math.abs(thetaArray[3] - thetaArray[1]); //difference btwn the second and fourth line

    // calculate appropriate coordinates
    x = -sensorDistance * Math.cos(Math.toRadians(thetaY / 2));
    y = -sensorDistance * Math.cos(Math.toRadians(thetaX / 2));

    //store in odometer
    odometer.setX(x);
    odometer.setY(y);
  }
  
  /**
   * TODO:
   */
  private void computeAngle() {
    double average;
    double deltaTheta1;
    double deltaTheta2;
    
    deltaTheta1 = 90 - (thetaArray[0] - 180) + (thetaY/2);
    deltaTheta2 = 90 - (thetaArray[1] - 180) + (thetaX/2);
    
    average = (deltaTheta1 + deltaTheta2) / 2;
    
    odometer.setTheta(average);
    
  }

  public float getLightSensor() {
    return lightSensorValue;
  }

}
