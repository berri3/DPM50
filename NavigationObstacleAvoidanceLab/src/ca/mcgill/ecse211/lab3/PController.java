package ca.mcgill.ecse211.lab3;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 135; //default motor speed
  private static final int FILTER_OUT = 50; //used for the filter below

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  private int distError;
  private int correction; //correction to be applied to the default motor speed
  private int deltaSpeed; //variable delta speed to be applied to the wheels

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0; //initialize the filter control to 0

//    NavigationObstacleAvoidanceLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
//    NavigationObstacleAvoidanceLab.rightMotor.setSpeed(MOTOR_SPEED);
//    NavigationObstacleAvoidanceLab.leftMotor.forward();
//    NavigationObstacleAvoidanceLab.rightMotor.forward();
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
      this.distance = 300; //capped at 300
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    //calculate the error based on the measured and filtered distance
    //versus the wanted distance
    distError = bandCenter - this.distance; 
    
    
    //error within acceptable limit; move forward normally
    if (Math.abs(distError) <= bandWidth){
    	NavigationObstacleAvoidanceLab.leftMotor.setSpeed(MOTOR_SPEED); // Start robot moving forward
    	NavigationObstacleAvoidanceLab.rightMotor.setSpeed(MOTOR_SPEED);
    	NavigationObstacleAvoidanceLab.leftMotor.forward();
    	NavigationObstacleAvoidanceLab.rightMotor.forward();
    }
    
    //else if not within acceptable error, adjust the two motors accordingly to the amplitude of the error
    //implement a function as input the error and as output the correction to increase or reduce
    
    //Note: two different cases for too close/far since the behaviour is programmed differently
    else if(distError > 0){ //too close
    	
    	//correction based on the error times a factor (8)
    	correction = Math.abs(distError) * 8;
    	
    	if (correction < 120) //capping the speed at 120 to avoid the robot running too fast
    		//robot not too fast: the deltaSpeed is the correction
    		deltaSpeed = correction;
    	else //too fast of a correction; set the change to 120
    		deltaSpeed = 120;
    	
    	NavigationObstacleAvoidanceLab.leftMotor.setSpeed(MOTOR_SPEED + deltaSpeed);
    	NavigationObstacleAvoidanceLab.rightMotor.setSpeed(MOTOR_SPEED - deltaSpeed);
    	NavigationObstacleAvoidanceLab.leftMotor.forward();
    	NavigationObstacleAvoidanceLab.rightMotor.forward();
    }
    else if(distError < 0){  //turn towards the wall
    	//correction based on the error times a factor (6)
    	correction = Math.abs(distError) * 6;
    	
    	if (Math.abs(correction)< 100) //capping the speed at 100
    		deltaSpeed = correction;
    	else
    		deltaSpeed = 100;
    	if (this.distance > 50) { //if making a large angle turn (to get around block)
    		//reduce the left motor a bit less than the DELTASPD since the robot
        	//often turns too much and makes a u-turn instead
    		NavigationObstacleAvoidanceLab.leftMotor.setSpeed((int) (MOTOR_SPEED + deltaSpeed/10));
    	}
    	else { //if going towards wall just to adjust distance
    		NavigationObstacleAvoidanceLab.leftMotor.setSpeed((int) (MOTOR_SPEED - deltaSpeed));
    	}
    	NavigationObstacleAvoidanceLab.rightMotor.setSpeed(MOTOR_SPEED + deltaSpeed);
    	NavigationObstacleAvoidanceLab.leftMotor.forward();
    	NavigationObstacleAvoidanceLab.rightMotor.forward();
    	
    }

  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
