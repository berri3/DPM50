package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;

public class UltrasonicLocalizer extends Thread{
	//TODO: enum
	public enum LocalizationType {FALLING_EDGE, RISING_EDGE};
	
	private LocalizationType localizationType;
	
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
	private int filteredDistance; //as filtered
	
	//filter stuff
	private int filterControl;
	
	//some constants (as per the main class; refer for description)
	private int forwardSpeed = LocalizationLab.MOTOR_HIGH;
	private int rotateSpeed = LocalizationLab.ROTATE_SPEED;
	private int acceleration = LocalizationLab.MOTOR_ACCELERATION;
	//TODO: added(?)
	private int threshold = LocalizationLab.THRESHOLD;
	private int noiseMargin = LocalizationLab.NOISE_MARGIN;
	private int filterOut = LocalizationLab.FILTER_OUT;
	
	
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
	
	public void localize(LocalizationType aLocalizationType){	
		localizationType = aLocalizationType;
		this.start();
		
	}
	
	public void run(){
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
		while (filterDistance(currentDistance) > threshold - noiseMargin ) {
			rotateCCW();
		}
		while (filterDistance(currentDistance) < threshold ) {
			rotateCCW();
		}
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaA = odometer.getTheta();
		
		//getting thetaB: angle of vertical wall
		//rotate CW to get angle B (where thetaA < thetaB)
		while (filterDistance(currentDistance) > threshold - noiseMargin ) {
			rotateCW();
		}
		while (filterDistance(currentDistance) < threshold ) {
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
		while (filterDistance(currentDistance) < threshold + noiseMargin ) {
			rotateCW();
		}
		while (filterDistance(currentDistance) > threshold ) {
			rotateCW();
		}
		// stop the motors and return the angle
		Sound.playNote(Sound.PIANO, 440, 200);
		stopMotors();
		
		//get the angle from the odometer; set as thetaA
		thetaA = odometer.getTheta();
		
		//getting thetaB: angle of vertical wall
		//rotate CCW to get angle B (where thetaA < thetaB)
		while (filterDistance(currentDistance) < threshold + noiseMargin ) {
			rotateCCW();
		}
		while (filterDistance(currentDistance) > threshold ) {
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
			deltaTheta = 45 - (thetaA + thetaB)/2.0;
		} 
		else {
			deltaTheta = 255 - (thetaA + thetaB)/2.0;
		}
		//Sound.playNote(Sound.PIANO, 880, 200);
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
	
	private int filterDistance(int distance){
	    // rudimentary filter - toss out invalid samples corresponding to null
	    // signal.
	    // (n.b. this was not included in the Bang-bang controller, but easily
	    // could have).
	    //
		filteredDistance = distance;
	    if (distance >= 255 && filterControl < filterOut) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	    } else if (distance >= 255) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      filteredDistance = 300; //capped at 300
	    } else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      filterControl = 0;
	      filteredDistance = distance;
	    }
	    return filteredDistance;
	}

	/**
	 *sets the distance as read by the US and by the usPoller
	 * 
	 */
	public void setDistance(int distance) {
		currentDistance = distance;
	}
	
	public int getFilteredDistance() {
		return filteredDistance;
	}
}
