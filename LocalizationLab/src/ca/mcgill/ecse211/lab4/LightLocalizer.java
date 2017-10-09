package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	
	//motors and stuff
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor sensorMotor; //TODO: not necessary(?)
	private Port lightPort;
	private Odometer odometer;
	private Navigation navigation;
	
	//initialize light sensor
	//2. get an instance of the sensor using the port
	private final SensorModes myLight = new EV3ColorSensor(lightPort);
	//3. get instance of sensor in specified measurement mode
	private final SampleProvider myLightSample = myLight.getMode("Red");
	//4. allocate memory buffer to for received data
	private float[] sampleLight = new float[myLightSample.sampleSize()];
	
	//position variables
	private double thetaA;
	private double thetaB;
	private double deltaTheta;
	
	//distance
	private int currentDistance; //as read by the US and given by the USPoller
	
	//light sensor value
	private float lightSensorValue;
	
	//some constants (as per the main class; refer for description)
	private int forwardSpeed = LocalizationLab.MOTOR_HIGH;
	private int rotateSpeed = LocalizationLab.ROTATE_SPEED;
	private int acceleration = LocalizationLab.MOTOR_ACCELERATION;
	//TODO: added(?)
	private int threshold = LocalizationLab.THRESHOLD;
	private double noiseMargin = LocalizationLab.NOISE_MARGIN;
	
	
	//constructor
	public LightLocalizer(NXTRegulatedMotor leftMotor, 
			NXTRegulatedMotor rightMotor, 
			EV3MediumRegulatedMotor sensorMotor, 
			Odometer odometer,
			Navigation aNavigation,
			Port aLightPort) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.navigation = aNavigation;
		this.lightPort = aLightPort;
	}
	
	public void run(){
		
		myLight.fetchSample(sampleLight, 0); //store a data sample in array sampleLight starting at index 0
	    lightSensorValue = sampleLight[0]; //store that value in a variable
	    
	    
	}
	
	
	public float getLightSensor() {
		return lightSensorValue;
	}
	
	

}
