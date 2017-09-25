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
  private static final double BLACK_LINE = 13.0; //value to know a black line is read
  
  
  private Odometer odometer;
  private static float lightSensorValue;
  
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
      lightSensorValue = sampleLight[0];
      if(lightSensorValue >= BLACK_LINE && !readLine) {
    	  
    	  //play sound
    	  //Sound.playNote(Sound.PIANO, 400, 200);
    	  //passed a line: increment the line counter
    	  counter ++;
    	  
    	  
    	  switch (counter) {
    	  	  case 1:
    	  		  odometer.setY(0); //crossed the first line
    	  		  initialYPos = odometer.getY();
    	  		  Sound.playNote(Sound.PIANO, 523, 200);
    	  		  break;
    	  	  case 4:
    	  		  odometer.setX(0);
    	  		  initialXPos = odometer.getX();
    	  		  Sound.playNote(Sound.PIANO, 523, 200);
    	  		  break;
    		  case 7: case 10:
    			  initialXPos = odometer.getX(); //first time sensor sees the line for each heading,
    			  initialYPos = odometer.getY(); //take the odometer position and use it as reference
    		  								     //for the future line crossings for the same heading
    			  Sound.playNote(Sound.PIANO, 523, 200);
    			  break;
    		  case 2: case 3: //robot moving in positive Y
    			  //add a tile length to the initial measurement
    			  //the new measurement should be the correct one, pass it on to the odometer.
    			  initialYPos += TILE_LENGTH;
    			  odometer.setY(initialYPos);
    			  Sound.playNote(Sound.PIANO, 659, 200);
    			  break;
    		  case 8: case 9: //robot moving in negative Y
    			  initialYPos -= TILE_LENGTH;
    			  odometer.setY(initialYPos);
    			  Sound.playNote(Sound.PIANO, 784, 200);
    			  break;
    		  case 5: case 6: //robot moving in positive X
    			  initialXPos += TILE_LENGTH;
    			  odometer.setX(initialXPos);
    			  Sound.playNote(Sound.PIANO, 932, 200);
    			  break;
    		  case 11: case 12: //robot moving in negative X
    			  initialXPos -= TILE_LENGTH;
    			  odometer.setX(initialXPos); 
    			  Sound.playNote(Sound.PIANO, 1047, 200);
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
  
  public static float returnLightSensor() {
	  return lightSensorValue;
  }
}
