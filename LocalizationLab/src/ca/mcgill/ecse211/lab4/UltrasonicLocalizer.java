package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;

public class UltrasonicLocalizer extends Thread{
	//TODO: enum
	public enum LocalizationType {FALLING_EDGE, RISING_EDGE}

	private static final int fallingThetaError = -20;  //TODO
	private static final int risingThetaError = -20;
	
	private LocalizationType localizationType;
	
	//motors and stuff
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor sensorMotor; //TODO: not necessary(?)
	private Odometer odometer;
	private Navigation navigation;
	private UltrasonicPoller usPoller;
	
	//position variables
	private double thetaA;
	private double thetaB;
	private double deltaTheta;
	
	//distance TODO:
//	private int currentDistance; //as read by the US and given by the USPoller
//	private int filteredDistance; //as filtered
	
	//filter stuff
	private int filterControl;
	
	//boolean
	private boolean detectedWall;
	
	//some constants (as per the main class; refer for description)
	private int forwardSpeed = LocalizationLab.MOTOR_HIGH;
	private int rotateSpeed = LocalizationLab.ROTATE_SPEED;
	private int acceleration = LocalizationLab.MOTOR_ACCELERATION;
	private int defaultAcceleration = LocalizationLab.DEFAULT_ACCELERATION;
	//TODO: added(?)
	private int threshold = LocalizationLab.THRESHOLD;
	private int noiseMargin = LocalizationLab.NOISE_MARGIN;
	private int filterOut = LocalizationLab.FILTER_OUT;

	
	
	
	//constructor
	public UltrasonicLocalizer(NXTRegulatedMotor leftMotor, 
			NXTRegulatedMotor rightMotor, 
			EV3MediumRegulatedMotor sensorMotor, 
			Odometer odometer,
			Navigation aNavigation,
			UltrasonicPoller usPoller) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.navigation = aNavigation;
		this.usPoller = usPoller;
	}
	
	public void localize(LocalizationType aLocalizationType){	
		localizationType = aLocalizationType;
		localize();
		
	}
	
	public void localize(){
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		filterControl = 0;
		
		if(localizationType == LocalizationType.RISING_EDGE){
			risingEdge();
		}
		else if(localizationType == LocalizationType.FALLING_EDGE){
			fallingEdge();
		}
		else{ //default or invalid option
			
		}
		
		//compute deltaTheta
		computeDeltaTheta();
		
		//adjust the odometer's theta value 
		double currentTheta = odometer.getTheta();
		odometer.setTheta(currentTheta + deltaTheta);
		
		//turn the robot to the *hopefully* appropriate origin
		navigation.turnTo(0);
	}
	
	public void risingEdge() {
		//getting thetaA: angle of horizontal wall
		//rotate CCW to get angle A (where thetaA < thetaB)
		
		rotateCCW();
		while (usPoller.getDistance() > threshold - noiseMargin );
		rotateCCW();
		while (usPoller.getDistance() < threshold ); //wait while it's still seeing a wall
		
		//but now it rises.
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaA = odometer.getTheta();
		
		//getting thetaB: angle of vertical wall
		//rotate CW to get angle B (where thetaA < thetaB)

		rotateCW();
		try {
			Thread.sleep(2500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		while (usPoller.getDistance() < threshold );
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaB = odometer.getTheta();
	}
	
	//detecting when a wall starts to be seen
	public void fallingEdge() {
		//getting thetaA: angle of horizontal wall
		//rotate CW to get angle A (where thetaA < thetaB)
//		detectedWall = false;
		rotateCW();
		while (usPoller.getDistance() < threshold + noiseMargin ); //wait while it is facing wall
		
		rotateCW();
		while (usPoller.getDistance() > threshold ); //wait while it's facing out
		
		//when it senses the wall at first:
		
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaA = odometer.getTheta();
		
//		//TODO:
//		detectedWall = true;
		
		//getting thetaB: angle of vertical wall
		//rotate CCW to get angle B (where thetaA < thetaB)
		rotateCCW();
		try {
			Thread.sleep(2500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		while (usPoller.getDistance() < threshold + noiseMargin );
		rotateCCW();
		
		while (usPoller.getDistance() > threshold); 
		
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaB = odometer.getTheta();
	}
	
	private double computeDeltaTheta(){
		
		if(localizationType == LocalizationType.FALLING_EDGE){
			//case 1
			if(thetaA > thetaB){
				deltaTheta = 45 - (thetaA + thetaB)/2.0;
			}
			
			//case 2
			else {
				deltaTheta = (255 + fallingThetaError) - ((thetaA + thetaB)/2.0);
			}
			//Sound.playNote(Sound.PIANO, 880, 200);
		}
		else{
			//case 1
			if(thetaA > thetaB){
				deltaTheta = 45 - (thetaA + thetaB)/2.0;
			}
			
			//case 2
			else {
				deltaTheta = (255 + risingThetaError) - ((thetaA + thetaB)/2.0);
			}
			//Sound.playNote(Sound.PIANO, 880, 200);
			
		}
		return deltaTheta;
	}
	
	private void rotateCW(){
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		leftMotor.forward();
		rightMotor.backward();
		
	}
	
	private void rotateCCW(){
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		leftMotor.backward();
		rightMotor.forward();
	}
	
	private void stopMotors(){
		leftMotor.setAcceleration(defaultAcceleration);
		rightMotor.setAcceleration(defaultAcceleration);
		leftMotor.stop(true);
		rightMotor.stop();
	}
	
	
	//TODO:
//	private int filterDistance(int distance){
//	    // rudimentary filter - toss out invalid samples corresponding to null
//	    // signal.
//	    // (n.b. this was not included in the Bang-bang controller, but easily
//	    // could have).
//	    //
//		filteredDistance = distance;
//	    if (distance >= 255 && filterControl < filterOut) {
//	      // bad value, do not set the distance var, however do increment the
//	      // filter value
//	      filterControl++;
//	    } else if (distance >= 255) {
//	      // We have repeated large values, so there must actually be nothing
//	      // there: leave the distance alone
//	      filteredDistance = 300; //capped at 300
//	    } else {
//	      // distance went below 255: reset filter and leave
//	      // distance alone.
//	      filterControl = 0;
//	      filteredDistance = distance;
//	    }
//	    return filteredDistance;
//	}
	
//TODO:
//	/**
//	 *sets the distance as read by the US and by the usPoller
//	 * 
//	 */
//	public void setDistance(int distance) {
//		currentDistance = distance;
//	}
//	
//	public int getFilteredDistance() {
//		return filteredDistance;
//	}
//	
//	public int getCurrentDistance() {
//		return currentDistance;
//		
//	}
}
