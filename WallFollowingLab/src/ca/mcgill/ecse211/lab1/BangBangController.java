package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter; //constant offset from the wall
  private final int bandwidth; //width of dead band (allowed error (?))
//private final int motorLow;
  private final int motorHigh; //normal forward motor speed
  
  private int distance; //Measured and filtered distance from the wall
  private int distError; //Calculated error from the measured distance
  private int filterControl; //used for the filter
  
  private static final int DELTASPD = 75; //default change in motor speed when adjusting itself
  private static final int FILTER_OUT = 20; //used for the below filter

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
  //this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    this.filterControl = 0; //initialize the filter control to 0
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	  //Filter which filters out extreme data
	  if (distance >= 255 && filterControl < FILTER_OUT) {
		  // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	  } 
	  else if (distance >= 255) {
		  // We have repeated large values, so there must actually be nothing
		  // there: put the distance to 255
		  // the above is done since there seems to be problems with extra large values of distance
		  this.distance = 255;
	  } 
	  else {
		  // distance went below 255: reset filter and leave
		  // distance alone.
		  filterControl = 0;
		  this.distance = distance;
	  }
	  
	  //computing the difference between the wanted value vs the actual value of distance
	  distError = bandCenter - this.distance;
    
//    if (distance > 200) { //really too far; probably because the wall is ending
//    	WallFollowingLab.leftMotor.setSpeed(motorReallyHigh);
//    	WallFollowingLab.rightMotor.setSpeed(motorReallyHigh);
//    	WallFollowingLab.leftMotor.backward();
//    	WallFollowingLab.rightMotor.forward();
//    }
    
    if (Math.abs(distError) <= bandwidth){ //within acceptable error
    	//keep moving the robot forward at the original speed
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    else if(distError > 0){ //too close; turn away from the wall
    	WallFollowingLab.leftMotor.setSpeed(motorHigh + DELTASPD);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh - DELTASPD);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    else if(distError < 0){ //too far; turn towards the wall
    	//reduce the left motor a bit less than the DELTASPD since the robot
    	//often turns too much and makes a u-turn instead
    	WallFollowingLab.leftMotor.setSpeed(motorHigh - (DELTASPD/3));
    	WallFollowingLab.rightMotor.setSpeed(motorHigh + DELTASPD);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
