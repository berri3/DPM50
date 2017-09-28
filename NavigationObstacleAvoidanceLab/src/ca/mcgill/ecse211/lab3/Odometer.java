package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
  // robot position
  private double x; //current x coordinate wrt the origin (intersection of the closest 2 black lines
  private double y; //current y coordinate wrt the origin (intersection of the closest 2 black lines
  private double theta; //current angle with origin the positive y axis and going + CW
  private int leftMotorTachoCount; //current tachometer count for the left motor
  private int leftMotorLastTC; //previous tachometer count for the left motor
  private int rightMotorTachoCount; //current tachometer count for the right motor
  private int rightMotorLastTC; //previous tachometer count for the right motor
  
  private double distL, distR, deltaD, deltaTRad, deltaT, dX, dY; //used later for computing x, y, and theta
    
  private double wheelR = NavigationObstacleAvoidanceLab.WHEEL_RADIUS;
  private double wheelW = NavigationObstacleAvoidanceLab.TRACK;
  private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/
  
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  
  private Object lock; /*lock object for mutual exclusion*/

  // default constructor
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    lock = new Object();
  }

  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
    
    leftMotor.resetTachoCount(); //reset counters
    rightMotor.resetTachoCount();
    leftMotorLastTC = leftMotor.getTachoCount(); //should be 0°, but just in case :)
    rightMotorLastTC = rightMotor.getTachoCount();

    while (true) {
    	
      updateStart = System.currentTimeMillis();
      
      //get the new tachometer count
      leftMotorTachoCount = leftMotor.getTachoCount(); 
      rightMotorTachoCount = rightMotor.getTachoCount();      
      
      //computing the displacement for each wheel by calculating the difference between the old and new
      distL = (Math.PI)*wheelR*(leftMotorTachoCount-leftMotorLastTC)/180;
      distR = (Math.PI)*wheelR*(rightMotorTachoCount-rightMotorLastTC)/180;
      
      //update the previous counts with the current counts
      leftMotorLastTC = leftMotorTachoCount;
      rightMotorLastTC = rightMotorTachoCount;
      
      //deltaD = (distL + distR)/2;
      deltaTRad = (distL - distR)/wheelW; //computing the change in angle (radians)
      deltaT = deltaTRad * 360 / (2*Math.PI); //radians -> degrees
      
      
      synchronized (lock) {
        /**
         * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
         * and theta in this block. Do not perform complex math
         * 
         */
    	 
    	 if(theta + deltaT >= 0) { //updating theta, in degrees
    		 theta = (theta + deltaT) % 360; //limit the range to [0, 359]
    	 }
    	 else //if negative angle, add 360, since we want the positive value
    		 theta = ((theta + deltaT) % 360) + 360;
    	 
         dX = distL*Math.sin(theta*Math.PI/180);
         dY = distR*Math.cos(theta*Math.PI/180);
         
         //update position
    	 x = x + dX;
         y = y + dY;
      }

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometer will be interrupted by
          // another thread
        }
      }
    }
  }

  public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = theta;
    }
  }

  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  // mutators
  public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /**
   * @return the leftMotorTachoCount
   */
  public int getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * @param leftMotorTachoCount the leftMotorTachoCount to set
   */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /**
   * @return the rightMotorTachoCount
   */
  public int getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

  /**
   * @param rightMotorTachoCount the rightMotorTachoCount to set
   */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}
