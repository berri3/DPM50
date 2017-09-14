package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

public class Printer extends Thread {

  //
  // In addition to the UltrasonicPoller, the printer thread also operates
  // in the background. Since the thread sleeps for 200 mS each time through
  // the loop, screen updating is limited to 5 Hz.
  //

  private UltrasonicController cont;
  private final int option;
  
  private int leftMotorSpeed;
  private int rightMotorSpeed;

  public Printer(int option, UltrasonicController cont) {
    this.cont = cont;
    this.option = option;
  }

  public static TextLCD t = LocalEV3.get().getTextLCD(); // n.b. how the screen is accessed

  public void run() {
    while (true) { // operates continuously
      t.clear();
      t.drawString("Controller Type is... ", 0, 0); // print header
      if (this.option == Button.ID_LEFT)
        t.drawString("BangBang", 0, 1);
//     	t.drawString("Real: " , x, y);
      else if (this.option == Button.ID_RIGHT)
        t.drawString("P type", 0, 1);
//      t.drawString("Real: " , x, y);
      t.drawString("US Distance: " + cont.readUSDistance(), 0, 2); // print last US reading
      
      
      
      //print the speed of the two motors?
      leftMotorSpeed = WallFollowingLab.leftMotor.getRotationSpeed();
      rightMotorSpeed = WallFollowingLab.rightMotor.getRotationSpeed();
      
      
      t.drawString("Left: " + leftMotorSpeed, 0, 3);
      t.drawString("Right: " + rightMotorSpeed, 0, 4);
      
      try {
        Thread.sleep(200); // sleep for 200 mS
      } catch (Exception e) {
        System.out.println("Error: " + e.getMessage());
      }
    }
  }

  public static void printMainMenu() { // a static method for drawing
    t.clear(); // the screen at initialization
    t.drawString("left = bangbang", 0, 0);
    t.drawString("right = p type", 0, 1);
  }
}
