/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab3;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  
  private static final double TILE_LENGTH = 30.48; // as given per lab instructions
  private static final double BLACK_LINE = 13.0; //value corresponding to the signal returned
  												 //to the light sensor by a black line
  
  //position variables used to correct the robot's odometer with the theoretical measurements of the a tile
  private static double initialYPos;
  private static double initialXPos;
  
  private Odometer odometer;
  private static float lightSensorValue; //value read by the light sensor
  
  //initialize the light sensor
  //1. get an instance of the port used
  private static final Port portLight = LocalEV3.get().getPort("S1");
  //2. get an instance of the sensor using the port
  private static final SensorModes myLight = new EV3ColorSensor(portLight);
  //3. get instance of sensor in specified measurement mode
  private static final SampleProvider myLightSample = myLight.getMode("Red");
  //4. allocate memory buffer to for received data
  private static float[] sampleLight = new float[myLightSample.sampleSize()];
  
  // constructor
  public OdometryCorrection(Odometer odometer) {
    this.odometer = odometer;
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    int counter = 0; //counting how many lines the robot has crossed
    
    //wanted X, Y positions with respect to the measured TILE_LENGTH
    initialYPos = 0; //initialize to zero
    initialXPos = 0;
    
    //true if sensor is on a black line and has never acted upon it before
    //false if not on a black line or has already acted upon the same black line
    boolean readLine = false;
    
    while (true) {
      correctionStart = System.currentTimeMillis();

      myLight.fetchSample(sampleLight, 0); //store a data sample in array sampleLight starting at index 0
      lightSensorValue = sampleLight[0]; //store that value in a variable
      
      //if robot is on a black line for the first time
      if(lightSensorValue >= BLACK_LINE && !readLine) {
    	  
    	  //passed a line: increment the line counter
    	  counter ++;
    	  
    	  //different operations done depending on where we are/how many lines crossed
    	  switch (counter) {
    	  	  //1. going in the positive Y axis
    	  	  case 1:
    	  		  //we know that the first line crossed while going north is the Y origin (0)
    	  		  //set the Y position to 0
    	  		  odometer.setY(0);
    	  		  initialYPos = odometer.getY(); //store that value in the wanted Y position
    	  		  Sound.playNote(Sound.PIANO, 523, 200);
    	  		  break;
    		  case 2: case 3: //robot moving in positive Y
    			  //add a tile length to the initial measurement
    			  //the new measurement should be the correct one, pass it on to the odometer.
    			  initialYPos += TILE_LENGTH;
    			  odometer.setY(initialYPos);
    			  Sound.playNote(Sound.PIANO, 659, 200);
    			  break;
    			  
    		  //2. going in the positive X axis
    	  	  case 4:
    	  		  odometer.setX(0); //first time crossing a vertical line: must be where X=0
    	  		  initialXPos = odometer.getX();
    	  		  Sound.playNote(Sound.PIANO, 523, 200);
    	  		  break;
       		  case 5: case 6: 
    			  initialXPos += TILE_LENGTH;
    			  odometer.setX(initialXPos);
    			  Sound.playNote(Sound.PIANO, 784, 200);
    			  break;
    		  
    		  //3. going in the negative Y axis
    		  case 7: case 10: //case 10 is going in the negative x axis
    			  initialXPos = odometer.getX(); //first time sensor sees the line for each heading,
    			  initialYPos = odometer.getY(); //take the odometer position and use it as reference
    		  								     //for the future line crossings for the same heading
    			  Sound.playNote(Sound.PIANO, 523, 200);
    			  break;
    		  case 8: case 9: //robot moving in negative Y
    			  initialYPos -= TILE_LENGTH;
    			  odometer.setY(initialYPos);
    			  Sound.playNote(Sound.PIANO, 932, 200);
    			  break;
    			  
    		  //4. going in the negative X axis
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
  
  public static float getLightSensor() {
	  return lightSensorValue;
  }
  
  public static double getInitialYPos() {
	  return initialYPos;
  }
  
  public static double getInitialXPos() {
	  return initialXPos;
  }
}
