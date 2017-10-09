package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;

public class UltrasonicLocalizer {
	//TODO: enum
	public enum LocalizationType {FALLING_EDGE, RISING_EDGE};
	
	//motors and stuff
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor sensorMotor; //TODO: not necessary(?)
	private Odometer odometer;
	private Navigation navigation;
	
	//position variables
	private double thetaA;
	private double thetaB;
	private double deltaTheta;
	
	//distance
	private int currentDistance; //as read by the US and given by the USPoller
	
	//some constants (as per the main class; refer for description)
	private int forwardSpeed = LocalizationLab.MOTOR_HIGH;
	private int rotateSpeed = LocalizationLab.ROTATE_SPEED;
	private int acceleration = LocalizationLab.MOTOR_ACCELERATION;
	//TODO: added(?)
	private int threshold = LocalizationLab.THRESHOLD;
	private double noiseMargin = LocalizationLab.NOISE_MARGIN;
	
	//constructor
	public UltrasonicLocalizer(NXTRegulatedMotor leftMotor, 
			NXTRegulatedMotor rightMotor, 
			EV3MediumRegulatedMotor sensorMotor, 
			Odometer odometer,
			Navigation aNavigation) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.navigation = aNavigation;
	}
	
	public void run(LocalizationType aLocalizationType){	
		
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
		
		if(aLocalizationType == LocalizationType.RISING_EDGE){
			risingEdge();
		}
		else if(aLocalizationType == LocalizationType.FALLING_EDGE){
			fallingEdge();
		}
		else{ //default or invalid option
			
		}
		
		//compute deltaTheta
		computeDeltaTheta();
		
		//adjust the odometer's theta value 
		odometer.setTheta(odometer.getTheta() + deltaTheta);
		
		//turn the robot to the *hopefully* appropriate origin
		navigation.turnTo(0);
	}
	
	public void risingEdge() {
		//getting thetaA: angle of horizontal wall
		//rotate CCW to get angle A (where thetaA < thetaB)
		while (currentDistance > threshold - noiseMargin ) {
			rotateCCW();
		}
		while (currentDistance < threshold ) {
			rotateCCW();
		}
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaA = odometer.getTheta();
		
		//getting thetaB: angle of vertical wall
		//rotate CW to get angle B (where thetaA < thetaB)
		while (currentDistance > threshold - noiseMargin ) {
			rotateCW();
		}
		while (currentDistance < threshold ) {
			rotateCW();
		}
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
		while (currentDistance < threshold + noiseMargin ) {
			rotateCW();
		}
		while (currentDistance > threshold ) {
			rotateCW();
		}
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaA = odometer.getTheta();
		
		//getting thetaB: angle of vertical wall
		//rotate CCW to get angle B (where thetaA < thetaB)
		while (currentDistance < threshold + noiseMargin ) {
			rotateCCW();
		}
		while (currentDistance > threshold ) {
			rotateCCW();
		}
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaB = odometer.getTheta();
	}
	
	private double computeDeltaTheta(){
		if(thetaA > thetaB){
			deltaTheta = 225 - (thetaA + thetaB)/2.0;
		} else {
			deltaTheta = 45 - (thetaA + thetaB)/2.0;
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
		rightMotor.stop(true);
		leftMotor.stop();
	}

	/**
	 *sets the distance as read by the US and by the usPoller
	 * 
	 */
	public void setDistance(int distance) {
		currentDistance = distance;
	}
	
	public int getDistance() {
		return currentDistance;
	}
}
