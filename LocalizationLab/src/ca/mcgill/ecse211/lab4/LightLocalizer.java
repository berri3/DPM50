package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread {
	
	//motors and stuff
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor sensorMotor; //TODO: not necessary(?)
	private Port lightPort;
	private Odometer odometer;
	private Navigation navigation;
	
	private SensorModes myLight;
	private float[] sampleLight;
	
//	//initialize light sensor
//	//2. get an instance of the sensor using the port
//	private final SensorModes myLight = new EV3ColorSensor(lightPort);
//	//3. get instance of sensor in specified measurement mode
//	private final SampleProvider myLightSample = myLight.getMode("Red");
//	//4. allocate memory buffer to for received data
//	private float[] sampleLight = new float[myLightSample.sampleSize()];
	
	
	
	//position variables
	private double thetaA;
	private double thetaB;
	private double deltaTheta;
	
	//distance
	private int currentDistance; //as read by the US and given by the USPoller
	
	//light sensor value
	private float lightSensorValue;
	
	//boolean
	private boolean readLine;
	
	//array
	private double[] thetaArray = new double[4];
	
	//some constants (as per the main class; refer for description)
	private int forwardSpeed = LocalizationLab.MOTOR_HIGH;
	private int rotateSpeed = LocalizationLab.ROTATE_SPEED;
	private int acceleration = LocalizationLab.MOTOR_ACCELERATION;
	private double wheelRadius = LocalizationLab.WHEEL_RADIUS;
	private double track = LocalizationLab.TRACK;
	//TODO: added(?)
	private int threshold = LocalizationLab.THRESHOLD;
	private double noiseMargin = LocalizationLab.NOISE_MARGIN;
	private double sensorDistance = LocalizationLab.SENSOR_DISTANCE;
	private double blackLine = LocalizationLab.BLACK_LINE;
	private int defaultAcceleration = LocalizationLab.DEFAULT_ACCELERATION;
	
	
	
	//constructor
	public LightLocalizer(NXTRegulatedMotor leftMotor, 
			NXTRegulatedMotor rightMotor, 
			EV3MediumRegulatedMotor sensorMotor, 
			Odometer odometer,
			Navigation aNavigation,
			SensorModes mylight,
			float[] sampleLight) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.navigation = aNavigation;
		this.myLight = mylight;
		this.sampleLight = sampleLight;
	}
	
	public void localize(){
		
	    turn360();
	    int counter = 0;
	    readLine = false;
	    while (counter < 4 && (leftMotor.isMoving() || rightMotor.isMoving())) {

	        myLight.fetchSample(sampleLight, 0); //store a data sample in array sampleLight starting at index 0
	        lightSensorValue = sampleLight[0]; //store that value in a variable
	        
	        //if robot is on a black line for the first time
	        if(lightSensorValue >= blackLine && !readLine) {
	      	  
	      	  	//passed a line: increment the line counter
	      	  
	        	thetaArray[counter] = odometer.getTheta();
	        	counter ++;
	        
	        
	        	readLine = true; //finished reading the line.
	        	Sound.playNote(Sound.PIANO, 880, 200);
	        }
	        else { //either actually not on a black line OR one one but already finished reading it
	        	readLine = false;
	        }
	    }

	    
	    while (Math.abs(odometer.getTheta() - 0) > 1); //wait until does full turn
	    //stop motors
	    stopMotors();
	    
	    
	    try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    
	    //compute & store correct position in odometer
	    computePosition();
	    
	    try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    
	    //travel to *hopefully* (0,0)
	    navigation.travelTo(0,0);
	    navigation.turnTo(0);
	    
	}
	
	
	
	
	public float getLightSensor() {
		return lightSensorValue;
	}
	
	private void turn360(){
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
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
	
	
	private void computePosition(){
		double thetaY;
		double thetaX;
		double x;
		double y;
		
		//compute thetaY and theta X
		thetaY = Math.abs(thetaArray[2] - thetaArray[0]);
		thetaX = Math.abs(thetaArray[3] - thetaArray[1]);
		
		x = -sensorDistance * Math.cos(Math.toRadians(thetaY/2));
		y = -sensorDistance * Math.cos(Math.toRadians(thetaX/2));
		
		odometer.setX(x);
		odometer.setY(y);
	}

}
