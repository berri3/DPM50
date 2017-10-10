package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread{
	
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
	//TODO: added(?)
	private int threshold = LocalizationLab.THRESHOLD;
	private double noiseMargin = LocalizationLab.NOISE_MARGIN;
	private double sensorDistance = LocalizationLab.SENSOR_DISTANCE;
	
	
	
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
	
	public void run(){

	    turn360();
	    int counter = 0;
	    while (counter < 4) {

	        myLight.fetchSample(sampleLight, 0); //store a data sample in array sampleLight starting at index 0
	        lightSensorValue = sampleLight[0]; //store that value in a variable
	        
	        //if robot is on a black line for the first time
	        if(lightSensorValue >= BLACK_LINE && !readLine) {
	      	  
	      	  	//passed a line: increment the line counter
	      	  
	        	thetaArray[counter] = odometer.getTheta();
	        	counter ++;
	        }
	    }
	    
	}
	
	
	
	
	public float getLightSensor() {
		return lightSensorValue;
	}
	
	private void turn360(){
		leftMotor.rotate(-360);
		rightMotor.rotate(360);
	}
	
	private void computePosition(){
		double thetaY;
		double thetaX;
		double x;
		double y;
		
		//compute thetaY and theta X
		thetaY = Math.abs(thetaArray[2] - thetaArray[0]);
		thetaX = Math.abs(thetaArray[3] - thetaArray[1]);
		
		x = -sensorDistance * Math.cos(thetaY/2);
		y = -sensorDistance * Math.cos(thetaX/2);
	}

}
