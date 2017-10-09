package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.UltrasonicLocalizer.LocalizationType;
import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationLab {

	//get instances of the motors and sensors with their respective ports
  public static final NXTRegulatedMotor leftMotor =
      new NXTRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final NXTRegulatedMotor rightMotor =
	  new NXTRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3MediumRegulatedMotor sensorMotor =
	  new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final Port usPort = LocalEV3.get().getPort("S1");
  //light sensor
  private static final Port lightPort = LocalEV3.get().getPort("S2");

  //some constants
  public static final double WHEEL_RADIUS = 2.1; //in cm
  public static final double TRACK = 12.5; //distance between the two wheels in cm
  public static final double TILE_LENGTH = 30.48; //in cm, as provided
  public static final int MOTOR_HIGH = 110; // Speed of the faster rotating wheel (deg/sec)
  public static final int ROTATE_SPEED = 75; //in deg/s
  public static final int MOTOR_ACCELERATION = 150; //in deg/s^2
  
  //TODO: added
  public static final int THRESHOLD = 30; //distance threshold before considering wall seen
  public static final double NOISE_MARGIN = 3;
  public static final long CORRECTION_PERIOD = 10;
  public static final double BLACK_LINE = 13.0; //value corresponding to the signal returned
  												 //to the light sensor by a black line

  public static void main(String[] args) {
    int buttonChoice;
    final TextLCD t = LocalEV3.get().getTextLCD();
    
    //set up US
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // this instance
    float[] usData = new float[usDistance.sampleSize()]; 
    
    //set up odometer
    Odometer odometer = new Odometer(leftMotor, rightMotor);
    
    //set up p-controller
    //PController pController = new PController(BANDCENTER, BANDWIDTH);
    
    //set up ultrasonic poller
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);
    
    //set up navigation 
    Navigation navigation = new Navigation(leftMotor,
    		rightMotor,
    		sensorMotor,
    		odometer);
    
    //set up localization
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer(leftMotor,
    		rightMotor,
    		sensorMotor,
    		odometer,
    		navigation);
    
    //set up localization
    LightLocalizer lightLocalizer = new LightLocalizer(leftMotor,
    		rightMotor,
    		sensorMotor,
    		odometer,
    		navigation, 
    		lightPort);
    
    //add the navigation to the poller
    usPoller.addNavigation(navigation);
    
    //set up display
    Display display = new Display(odometer, t, ultrasonicLocalizer, lightLocalizer);
    		
    do {
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString(" Light |   US   ", 0, 2);
      t.drawString("Locali-|Locali- ", 0, 3);
      t.drawString("  zer  |   zer  ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) { //do lightLocalizer
     

      odometer.start();
      display.start();

    } else { //do usLocalizer
      // clear the display
      t.clear();

      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString(" Rising| Falling", 0, 2);
      t.drawString("  edge |  edge  ", 0, 3);
      t.drawString("       |        ", 0, 4);
      
      buttonChoice = Button.waitForAnyPress();
      
      //start threads
      odometer.start();
      display.start();
      usPoller.start();
      
      if(buttonChoice == Button.ID_RIGHT){
    	  ultrasonicLocalizer.run(LocalizationType.FALLING_EDGE);
      }
      
      else{//start navigation
    	  ultrasonicLocalizer.run(LocalizationType.RISING_EDGE);
      }
      
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
