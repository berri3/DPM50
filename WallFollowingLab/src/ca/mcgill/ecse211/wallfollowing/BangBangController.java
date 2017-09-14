package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter; //constant offset from the wall
  private final int bandwidth; //width of dead band (allowed error (?))
  private final int motorLow;
  private final int motorHigh;  
  private int distance; //Distance from the wall(?)
  private int distError; //Calculated error from the measured distance
  private int filterControl;
  private static final int DELTASPD = 75;

  private static final int FILTER_OUT = 20;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    this.filterControl = 0;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	  // rudimentary filter - toss out invalid samples corresponding to null
	  // signal.
	  // (n.b. this was not included in the Bang-bang controller, but easily
	  // could have).
	  //
	  if (distance >= 255 && filterControl < FILTER_OUT) {
		  // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	  } 
	  else if (distance >= 255) {
		  // We have repeated large values, so there must actually be nothing
		  // there: leave the distance alone
		  this.distance = 255; //capped at 255
	  } 
	  else {
		  // distance went below 255: reset filter and leave
		  // distance alone.
		  filterControl = 0;
		  this.distance = distance;
	  }
	  distError = bandCenter - this.distance;
    
//    if (distance > 200) { //really too far; probably because the wall is ending
//    	WallFollowingLab.leftMotor.setSpeed(motorReallyHigh);
//    	WallFollowingLab.rightMotor.setSpeed(motorReallyHigh);
//    	WallFollowingLab.leftMotor.backward();
//    	WallFollowingLab.rightMotor.forward();
//    }
    
    if (Math.abs(distError) <= bandwidth){
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    else if(distError > 0){ //turn away from the wall
    	WallFollowingLab.leftMotor.setSpeed(motorHigh + DELTASPD);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh - DELTASPD);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    else if(distError < 0){ //turn towards the wall
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
