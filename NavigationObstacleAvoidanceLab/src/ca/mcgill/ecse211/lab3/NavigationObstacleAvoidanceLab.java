// Lab3.java

package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

public class NavigationObstacleAvoidanceLab {

  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
	  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3MediumRegulatedMotor sensorMotor =
	  new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final Port lightPort = LocalEV3.get().getPort("S2");

  public static final double WHEEL_RADIUS = 2.15; //in cm
  public static final double TRACK = 15; //distance between the two wheels in cm
  private static final int bandCenter = 37; // Offset from the wall (cm)
  private static final int bandWidth = 4; // Width of dead band (cm)
  private static final int motorHigh = 110; // Speed of the faster rotating wheel (deg/sec)

  public static void main(String[] args) {
    int buttonChoice;
    final TextLCD t = LocalEV3.get().getTextLCD();
    Odometer odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
    OdometryCorrection odometryCorrection = new OdometryCorrection(odometer);

    do {
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString(" Float | Drive  ", 0, 2);
      t.drawString("motors | in a   ", 0, 3);
      t.drawString("       | square ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {

      leftMotor.forward();
      leftMotor.flt();
      rightMotor.forward();
      rightMotor.flt();

      odometer.start();
      odometryDisplay.start();

    } else {
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("  No   | with   ", 0, 1);
      t.drawString(" corr- | corr-  ", 0, 2);
      t.drawString(" ection| ection ", 0, 3);
      t.drawString("       |        ", 0, 4);
      
      buttonChoice = Button.waitForAnyPress();
      
      odometer.start();
      odometryDisplay.start();
      
      if(buttonChoice == Button.ID_RIGHT){
        odometryCorrection.start();
      }
      
      // spawn a new Thread to avoid SquareDriver.drive() from blocking
      (new Thread() {
        public void run() {
          SquareDriver.drive(leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK);
        }
      }).start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
