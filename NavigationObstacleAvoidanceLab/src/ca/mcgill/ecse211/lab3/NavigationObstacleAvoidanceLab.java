// Lab3.java

package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import ca.mcgill.ecse211.lab3.PController;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationObstacleAvoidanceLab {

  public static final NXTRegulatedMotor leftMotor =
      new NXTRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final NXTRegulatedMotor rightMotor =
	  new NXTRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3MediumRegulatedMotor sensorMotor =
	  new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final Port usPort = LocalEV3.get().getPort("S1");
  //private static final Port lightPort = LocalEV3.get().getPort("S2");

  public static final double WHEEL_RADIUS = 2.1; //in cm
  public static final double TRACK = 12.5; //distance between the two wheels in cm
  public static final double TILE_LENGTH = 30.48;
  public static final int bandCenter = 37; // Offset from the wall (cm)
  public static final int bandWidth = 4; // Width of dead band (cm)
  public static final int motorHigh = 110; // Speed of the faster rotating wheel (deg/sec)
  public static final int ROTATE_SPEED = 75;
  public static final int motorAcceleration = 150; //
  public static final int THRESHOLD = 20; 
  public static final int ANGLE_THRESHOLD = 2; //in degrees; acceptable angle to stop avoidance

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
    
    //set up correction
    //OdometryCorrection odometryCorrection = new OdometryCorrection(odometer);
    
    //set up p-controller
    PController pController = new PController(bandCenter, bandWidth);
    
    //set up ultrasonic poller
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, pController);
    
    //set up navigation 
    Navigation navigation = new Navigation(leftMotor,
    		rightMotor,
    		sensorMotor,
    		odometer,
    		usPoller,
    		pController);
    
    //add the navigation to the poller
    usPoller.addNavigation(navigation);
    
    //set up display
    Display display = new Display(odometer, pController, t, navigation);
    		
    do {
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString(" Float | Do na-  ", 0, 2);
      t.drawString("motors | vigation", 0, 3);
      t.drawString("       |         ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {

      leftMotor.forward();
      leftMotor.flt();
      rightMotor.forward();
      rightMotor.flt();

      odometer.start();
      display.start();

    } else { //do navigation
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("  No   | with   ", 0, 1);
      t.drawString(" corr- | corr-  ", 0, 2);
      t.drawString(" ection| ection ", 0, 3);
      t.drawString(" Nav.  |No touch", 0, 4);
      
      buttonChoice = Button.waitForAnyPress();
      
      odometer.start();
      display.start();
      usPoller.start();
      
      if(buttonChoice == Button.ID_RIGHT){
        //odometryCorrection.start();
      }
      //start navigation
      navigation.start();
      
//      // spawn a new Thread to avoid SquareDriver.drive() from blocking
//      (new Thread() {
//        public void run() {
//          SquareDriver.drive(leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK);
//        }
//      }).start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
