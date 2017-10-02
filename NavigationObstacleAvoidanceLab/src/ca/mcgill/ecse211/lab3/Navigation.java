package ca.mcgill.ecse211.lab3;

import lejos.hardware.Sound;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Navigation extends Thread{
	
	//motors and stuff
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor sensorMotor;
	private Odometer odometer;
	private UltrasonicPoller usPoller;
	private PController controller;
	
	//position variables
	private double x;
	private double y;
	private double theta;
	
	//distance
	private int currentDistance;
	
	//some constants
	private double tileLength = NavigationObstacleAvoidanceLab.TILE_LENGTH;
	private int forwardSpeed = NavigationObstacleAvoidanceLab.MOTOR_HIGH;
	private double wheelRadius = NavigationObstacleAvoidanceLab.WHEEL_RADIUS;
	private double track = NavigationObstacleAvoidanceLab.TRACK;
	private int rotateSpeed = NavigationObstacleAvoidanceLab.ROTATE_SPEED;
	private int acceleration = NavigationObstacleAvoidanceLab.MOTOR_ACCELERATION;
	private int threshold = NavigationObstacleAvoidanceLab.THRESHOLD; //acceptable error for obstacle
	private int angleThreshold = NavigationObstacleAvoidanceLab.ANGLE_THRESHOLD;
	
	//boolean
	//private boolean isNavigating;
	
	
	//constructor
	public Navigation(NXTRegulatedMotor leftMotor, 
			NXTRegulatedMotor rightMotor, 
			EV3MediumRegulatedMotor sensorMotor, 
			Odometer odometer, 
			UltrasonicPoller usPoller,
			PController pController) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.usPoller = usPoller;
		this.controller = pController;
	}
	
	public void run() {
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		travelTo(0,3);
		travelTo(3,3);
	}
	
	void travelTo(double pointX, double pointY) {
		double minAngle, travelDistance, distanceX, distanceY, deltaX, deltaY; //angleDifference;
		
		//update current position TODO: does it impact stuff?
		x = odometer.getX();
		y = odometer.getY();
		
		//convert point coordinates to actual distance *in cm*
		distanceX = pointX*tileLength;
		distanceY = pointY*tileLength;
		
		//calculate distance to travel from current position in each direction
		deltaX = distanceX - odometer.getX();
		deltaY = distanceY - odometer.getY();
		
		//calculate pythagorean distance to travel
		travelDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		
		//turn towards the required angle 
		minAngle = computeHeading(distanceX, distanceY); //compute the required heading to turn to
		turnTo(minAngle);
		
		//travel
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);
		leftMotor.rotate(convertDistance(wheelRadius,travelDistance), true);
		//TODO: set to true if want avoidance
		rightMotor.rotate(convertDistance(wheelRadius, travelDistance), true);
		
		//obstacle check: do nothing if navigating and did not encounter obstacle yet
		//else take appropriate action	
		while(isNavigating() && currentDistance > threshold); //wait.
		//will stop waiting either b/c: robot has finished traveling OR seen an obstacle
		
		//if seen an obstacle while still moving
		//TODO: how is this gonna work for recursive calls(?)
		if(isNavigating()) {
			Sound.playNote(Sound.PIANO, 650, 500);
			double previousX, previousY;
			
			//decelerate to avoid slip; therefore not using .stop()
			leftMotor.setSpeed(0);
			rightMotor.setSpeed(0);
			leftMotor.forward();
			rightMotor.forward();
			leftMotor.flt();
			rightMotor.flt();
			
			previousX = odometer.getX();
			previousY = odometer.getY();
			
			//turn robot 90° right and prepare for wall follower
			leftMotor.setSpeed(rotateSpeed);
			rightMotor.setSpeed(rotateSpeed);
			leftMotor.rotate(convertAngle(wheelRadius, track, 90), true);
			rightMotor.rotate(-convertAngle(wheelRadius, track, 90), false);
			
			//correct position
			odometer.setX(previousX);
			odometer.setY(previousY);
			
			//put sensor at 45 CCW°
			//sensorMotor.setSpeed(25);
			sensorMotor.setSpeed(25);
			sensorMotor.rotateTo(-45);
			
			//to know when to stop the obstacle avoidance state, check if current angle is close
			//to previous heading.
			//update theta
			theta = odometer.getTheta();
			
			//continually compute the new required heading and see if we're close to it
			while (Math.abs(computeHeading(distanceX, distanceY) - odometer.getTheta())
					> angleThreshold) {
				// start p controller, continuously
				controller.processUSData(currentDistance);
				//P-controller should keep running until robot is at correct heading
			}
			
			//put sensor back straight
			sensorMotor.rotateTo(0);
			//if we have the correct heading, exit the while loop:
			//technically, a recursive call to travelTo should now work. #pray
			travelTo(pointX, pointY);
			
		}
		
		//TODO: stop(?)
//		leftMotor.stop(true);
//		rightMotor.stop(true);
	}
	
	//newAngle should be a value between 0 and 360
	void turnTo(double theta){
		double previousX, previousY;
		
		previousX = odometer.getX();
		previousY = odometer.getY();
		
		leftMotor.setSpeed(rotateSpeed);
	    rightMotor.setSpeed(rotateSpeed);
	    this.theta = odometer.getTheta();
	    
		double angleDifference = theta - this.theta;
		//turn to the right
		if ((0 <= angleDifference && angleDifference <= 180)){
			leftMotor.rotate(convertAngle(wheelRadius, track, angleDifference), true);
			rightMotor.rotate(-convertAngle(wheelRadius, track, angleDifference), false);
			Sound.playNote(Sound.PIANO, 440, 200);
		}
		
		else if( angleDifference < -180 ) {
			leftMotor.rotate(convertAngle(wheelRadius, track, angleDifference+360), true);
			rightMotor.rotate(-convertAngle(wheelRadius, track, angleDifference+360), false);
			Sound.playNote(Sound.PIANO, 500, 200);
		}
		//turn to the left
		else {
			leftMotor.rotate(-convertAngle(wheelRadius, track, angleDifference-180), true);
			rightMotor.rotate(convertAngle(wheelRadius, track, angleDifference-180), false);
			Sound.playNote(Sound.PIANO, 880, 200);
		}
		
		odometer.setX(previousX);
		odometer.setY(previousY);
	}
	
	
	//This method returns true if another thread has called travelTo() or turnTo()
	//and the method has yet to return; false otherwise.
	//TODO: something to do with avoidance(?)
	boolean isNavigating() {
		boolean isNavigating;
		isNavigating = leftMotor.isMoving() || rightMotor.isMoving();
		return isNavigating;
	}
	
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	//takes in distance (coordinates in cm, not points)
	private double computeHeading(double destinationX, double destinationY) {
		//calculate distance to travel from current position in each direction
		double deltaX, deltaY, angle;
		
		deltaX = destinationX - odometer.getX();
		deltaY = destinationY - odometer.getY();
		
		//takes in x value first since 0° is at the y axis
		angle = Math.atan2(deltaX, deltaY); //gets the angle in the correct quadrant [-180, 180]
		if(angle < 0) { //so that the range of theta stays [0, 2pi[ or [0°, 359°]
			angle += Math.PI*2;
		}
		
		//convert into degrees
		angle = Math.toDegrees(angle);
		
		return angle;
	}
	
	//sets the distance as read by the US and by the usPoller
	public void setDistance(int distance) {
		currentDistance = distance;
	}
	
	public int getDistance() {
		return currentDistance;
	}

}
