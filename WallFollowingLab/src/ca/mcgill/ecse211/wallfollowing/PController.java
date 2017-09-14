package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 135;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  private int distError; //error
  private int correction;
  private int deltaSpeed; //

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
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
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = 255; //capped at 255
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
//    this.distance = distance;
    distError = bandCenter - this.distance;
    
    if (Math.abs(distError) <= bandWidth){
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Start robot moving forward
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    
    
    //else if not within acceptable error, adjust the two motors accordingly to the amplitude of the error
    //implement function with input the error and output the deltaspd to increase or reduce
    
    else if(distError > 0){ //turn away from the wall{ 
    	
    	//correction based on the error times a factor (10)
    	correction = Math.abs(distError) * 8;
    	
    	if (correction < 120) //capping the speed at 85
    		deltaSpeed = correction;
    	else
    		deltaSpeed = 120;
    	
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + deltaSpeed);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - deltaSpeed);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
    else if(distError < 0){  //turn towards the wall
    	//correction based on the error times a factor (10)
    	correction = Math.abs(distError) * 6;
    	
    	if (Math.abs(correction)< 100) //capping the speed at 85
    		deltaSpeed = correction;
    	else
    		deltaSpeed = 100;
    	
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + deltaSpeed);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    	
    }

  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
