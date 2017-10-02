package ca.mcgill.ecse211.lab3;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Navigation extends Thread{
	
	//motors and stuff
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor sensorMotor;
	private Odometer odometer;
	private UltrasonicPoller usPoller;
	
	//position variables
	private double x;
	private double y;
	private double theta;
	
	//distance
	private int currentDistance;
	
	//some constants
	private double tileLength = NavigationObstacleAvoidanceLab.TILE_LENGTH;
	private int forwardSpeed = NavigationObstacleAvoidanceLab.motorHigh;
	private double wheelRadius = NavigationObstacleAvoidanceLab.WHEEL_RADIUS;
	private double track = NavigationObstacleAvoidanceLab.TRACK;
	private int rotateSpeed = NavigationObstacleAvoidanceLab.ROTATE_SPEED;
	private int acceleration = NavigationObstacleAvoidanceLab.motorAcceleration;
	private int threshold = NavigationObstacleAvoidanceLab.THRESHOLD; //acceptable error
	
	//boolean
	private boolean isNavigating;
	
	//constructor
	public Navigation(EV3LargeRegulatedMotor leftMotor, 
			EV3LargeRegulatedMotor rightMotor, 
			EV3MediumRegulatedMotor sensorMotor, 
			Odometer odometer, 
			UltrasonicPoller usPoller,
			PController pController) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.usPoller = usPoller;
	}
	
	public void run() {
		leftMotor.setAcceleration(acceleration);
		travelTo(1,1);
		travelTo(0,2);
		travelTo(2,2);
		travelTo(2,1);
		travelTo(1,0);
		
	}
	
	void travelTo(double pointX, double pointY) {
		double minAngle, travelDistance, distanceX, distanceY, deltaX, deltaY;
		
		//since robot is traveling now sike
		//isNavigating = true;
		
		//update current position
		x = odometer.getX();
		y = odometer.getY();
		
		//convert point coordinates to actual distance in cm
		distanceX = pointX*tileLength;
		distanceY = pointY*tileLength;
		
		//calculate distance to travel from current position in each direction
		deltaX = distanceX - x;
		deltaY = distanceY - y;
		
		//calculate pythagorean distance to travel
		travelDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
		
		//takes in x value first since 0° is at the y axis
		minAngle = Math.atan2(deltaX, deltaY); //gets the angle in the correct quadrant [-180, 180]
		if(minAngle < 0) { //so that the range of theta stays [0, 2pi[ or [0°, 359°]
			minAngle += Math.PI*2;
		}
		
		//convert into degrees
		minAngle = Math.toDegrees(minAngle);
		
		//turn towards the required angle 
		turnTo(minAngle);
		
		//travel
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);
		leftMotor.rotate(convertDistance(wheelRadius,travelDistance), true);
		rightMotor.rotate(convertDistance(wheelRadius, travelDistance), false);
		
//		//obstacle check: do nothing if navigating and did not encounter obstacle yet
//		//else take appropriate action	
//		while(isNavigating && currentDistance > threshold); //wait.
//		//will stop waiting either b/c: robot has finished traveling OR seen an obstacle
//		
//		//if seen an obstacle while still moving
//		//TODO: how is this gonna work for recursive calls(?)
//		if(isNavigating) {
//			//decelerate to avoid slip; therefore not using .stop()
//			leftMotor.setSpeed(0);
//			rightMotor.setSpeed(0);
//			leftMotor.forward();
//			rightMotor.forward();
//			leftMotor.flt();
//			rightMotor.flt();
//			
//			//turn robot 90° right and prepare for wall follower
//			leftMotor.rotate(convertAngle(wheelRadius, track, 90), true);
//			rightMotor.rotate(-convertAngle(wheelRadius, track, 90), false);
//			//put sensor at 45 CCW°
//			sensorMotor.rotate(-45);		
//			
//			//to know when to stop the obstacle avoidance state, check if current angle is close
//			//to previous heading.
//			while (true) {
//				
//			}
//			
//		}
		
		//TODO: stop(?)
		leftMotor.stop(true);
		rightMotor.stop(true);
		
		
		

	}
	
	//newAngle should be a value between 0 and 360
	void turnTo(double theta){
		
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
	}
	
	
	//This method returns true if another thread has called travelTo() or turnTo()
	//and the method has yet to return; false otherwise.
	//TODO: something to do with avoidance(?)
	boolean isNavigating() {
		isNavigating = leftMotor.isMoving() || rightMotor.isMoving();
		return isNavigating;
	}
	
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	//sets the distance as read by the US and by the usPoller
	public void setDistance(int distance) {
		currentDistance = distance;
	}
	
	public int getDistance() {
		return currentDistance;
	}

}
