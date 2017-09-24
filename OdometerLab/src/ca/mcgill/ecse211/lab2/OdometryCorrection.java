/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab2;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  
  private static final double TILE_LENGTH = 30.48; // as given per lab instructions
  private static final double BLACK_LINE = 0.18; //value to know a black line is read
  
  
  private Odometer odometer;
  
  //initialize the light sensor
  private static final Port portLight = LocalEV3.get().getPort("S1");
  private static final SensorModes myLight = new EV3ColorSensor(portLight);  
  private static final SampleProvider myLightSample = myLight.getMode("Red");
  private static float[] sampleLight = new float[myLightSample.sampleSize()];
  
  // constructor
  public OdometryCorrection(Odometer odometer) {
    this.odometer = odometer;
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    int counter = 0; //counting how many lines the robot has crossed
    
    double initialYPos = 0;
    double initialXPos = 0;
    boolean readLine = false; //if the sensor is reading a line
    
    while (true) {
      correctionStart = System.currentTimeMillis();

      //TODO Place correction implementation here
      myLight.fetchSample(sampleLight, 0); //store a sample in array sampleLight starting at index 0
      if((sampleLight[0] <= BLACK_LINE) && !readLine) {
    	  
    	  //play sound
    	  Sound.playNote(Sound.FLUTE, 400, 200);
    	  //passed a line: increment the line counter
    	  counter ++;
    	  
    	  
    	  switch (counter) {
    		  case 1: case 4: case 7: case 10:
    			  initialXPos = odometer.getX(); //first time sensor sees the line for each heading,
    			  initialYPos = odometer.getY(); //take the odometer position and use it as reference
    		  								     //for the future line crossings for the same heading
    			  break;
    		  case 2: case 3: //robot moving in positive Y
    			  //add a tile length to the initial measurement
    			  //the new measurement should be the correct one, pass it on to the odometer.
    			  initialYPos += TILE_LENGTH;
    			  odometer.setY(initialYPos);
    			  break;
    			  
    		  case 8: case 9: //robot moving in negative Y
    			  initialYPos -= TILE_LENGTH;
    			  odometer.setY(initialYPos);
    			  break;
    		  case 5: case 6: //robot moving in positive X
    			  initialXPos += TILE_LENGTH;
    			  odometer.setX(initialXPos);
    			  break;
    		  case 11: case 12: //robot moving in negative X
    			  initialXPos -= TILE_LENGTH;
    			  odometer.setX(initialXPos);
    			  break;
    		  default: //in any other case, default behaviour.
    			  initialXPos = odometer.getX();
    			  initialYPos = odometer.getY();
    	  }
    	  readLine = true; //finished reading the line.
      }
      else { //either actually not on a black line OR one one but already finished reading it
    	  readLine = false;
      }
    	  
      
      
      
      
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometry correction will be
          // interrupted by another thread
        }
      }
    }
  }
}
