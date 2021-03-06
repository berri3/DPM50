package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.UltrasonicLocalizer.LocalizationType;
import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationLab {

  // get instances of the motors and sensors with their respective ports
  public static final NXTRegulatedMotor leftMotor =
      new NXTRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final NXTRegulatedMotor rightMotor =
      new NXTRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3MediumRegulatedMotor sensorMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final Port usPort = LocalEV3.get().getPort("S1"); // ultrasonic sensor
  private static final Port lightPort = LocalEV3.get().getPort("S2"); // light sensor

  // some constants
  public static final double WHEEL_RADIUS = 2.1; // in cm
  public static final double TRACK = 14.1; // distance between the two wheels in cm
  public static final double TILE_LENGTH = 30.48; // in cm, as provided
  public static final int MOTOR_HIGH = 110; // Default forward speed (deg/sec)
  public static final int ROTATE_SPEED = 75; // in deg/s
  public static final int MOTOR_ACCELERATION = 150; // in deg/s^2
  public static final int DEFAULT_ACCELERATION = 6000; // as set by the EV3/leJOS
  public static final int THRESHOLD = 50; // distance threshold before considering wall seen (cm)
  public static final int NOISE_MARGIN = 20; // to account for the noise in the usLocalizer (cm)
  public static final long CORRECTION_PERIOD = 10;
  public static final double BLACK_LINE = 12.5; // value corresponding to the signal returned
                                                // to the light sensor by a black line
  public static final double SENSOR_DISTANCE = 13.0; // distance between the light sensor and the
                                                     // robot's center of rotation (cm)
  public static final int FILTER_OUT = 50; // used to filter the US

  public static void main(String[] args) {
    int buttonChoice;
    final TextLCD t = LocalEV3.get().getTextLCD();

    // set up US
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // this instance
    float[] usData = new float[usDistance.sampleSize()];

    // set up light sensor
    // 2. get an instance of the sensor using the port
    SensorModes myLight = new EV3ColorSensor(lightPort);
    // 3. get instance of sensor in specified measurement mode
    SampleProvider myLightSample = myLight.getMode("Red");
    // 4. allocate memory buffer to for received data
    float[] sampleLight = new float[myLightSample.sampleSize()];

    // set up odometer
    Odometer odometer = new Odometer(leftMotor, rightMotor);

    // set up ultrasonic poller
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData);

    // set up navigation
    Navigation navigation = new Navigation(leftMotor, rightMotor, sensorMotor, odometer);

    // set up ultrasonic localization
    UltrasonicLocalizer ultrasonicLocalizer =
        new UltrasonicLocalizer(leftMotor, rightMotor, sensorMotor, odometer, navigation, usPoller);

    // set up light localization
    LightLocalizer lightLocalizer =
        new LightLocalizer(leftMotor, rightMotor, odometer, navigation, myLight, sampleLight);

    // set up display
    Display display = new Display(odometer, t, usPoller, lightLocalizer);

    do {
      // clear the display
      t.clear();

      // ask the user whether to only test the light localizer or also the US.
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString(" Light |   US   ", 0, 2);
      t.drawString("Locali-|Locali- ", 0, 3);
      t.drawString("  zer  |   zer  ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) { // do lightLocalizer
      odometer.start();
      display.start();

      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(Navigation.convertAngle(-WHEEL_RADIUS, TRACK, 360), true);
      rightMotor.rotate(Navigation.convertAngle(WHEEL_RADIUS, TRACK, 360), false);

      leftMotor.setAcceleration(DEFAULT_ACCELERATION);
      rightMotor.setAcceleration(DEFAULT_ACCELERATION);
      leftMotor.stop(true);
      rightMotor.stop();

    } else { // testing: just turn 360�
      // clear the display
      t.clear();

      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString(" Rising| Falling", 0, 2);
      t.drawString("  edge |  edge  ", 0, 3);
      t.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress();

      // start threads
      odometer.start();
      display.start();
      usPoller.start();

      if (buttonChoice == Button.ID_RIGHT) {
        ultrasonicLocalizer.localize(LocalizationType.FALLING_EDGE);
      }

      else {
        ultrasonicLocalizer.localize(LocalizationType.RISING_EDGE);
      }

    }

    do {
      // clear the display
      t.clear();

      // ask the user whether to only test the light localizer or also the US.
      t.drawString("                ", 0, 0);
      t.drawString("usLocalizer done", 0, 1);
      t.drawString(" press enter to ", 0, 2);
      t.drawString("    continue    ", 0, 3);
      t.drawString("   (or down)    ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_DOWN); // wait for user to
                                                                                 // start light
                                                                                 // localization
    if (buttonChoice == Button.ID_ENTER) { // do not fix position
      lightLocalizer.localize(false);
    }

    else {
      lightLocalizer.localize(true); // fix that position
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
