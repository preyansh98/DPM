/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	private static final double LineIntensity = 85; // have to find this by trial and error?
	private static final double SquareTile = 30.48;
	private static final double SensorOffset = 4.3;
	// Initializing the color sensor
	SampleProvider samples;
	private float[] sampleList;
	private float lightPoll;
	EV3ColorSensor colSensor;
	double position[] = new double[3]; // an array to hold the XYT

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection(Odometer odometer, EV3ColorSensor colSensor) throws OdometerExceptions {
		this.odometer = odometer;
		this.colSensor = colSensor;
		colSensor.setCurrentMode("Red");
		this.samples = colSensor.getMode("Red");
		this.sampleList = new float[colSensor.sampleSize()];
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	   double counterYF = 0, counterXR= 0;
	   boolean pathComplete = false; 
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();
			

			// TODO Trigger correction (When do I have information to correct?)

			

			// if(colSensor.isFloodlightOn() == false) {
			// colSensor.setFloodlight(Color.RED); //floodlight in reflect mode. Color.RED.
			// colSensor.setFloodlight(true); //floodlight is now on
			// }
			//

			// Then fetch the sample from the red mode
			colSensor.fetchSample(sampleList, 0); // new array so offset is 0
			// since in thread, continously updates sampleList at [0]

			lightPoll = sampleList[0] * 1000; // value was normalized between 0 and 1: so we changed it to

			LCD.drawString("light val: " + lightPoll, 0, 4);
			
			   
			if (lightPoll < LineIntensity) {
				// code for when the black line is detected.
				Sound.beep();

				position = odometer.getXYT();
				double theta = position[2];// was 3 changed to 2
				
			
				if (theta > 315 || theta < 45) { 
					// should be at 0
			    	//every time it detects a black line it knows its passed another square tile
					
					//we need to use a boolean so that this doesn't run at the end. 
					odometer.setTheta(0);
					odometer.setY(SquareTile*counterYF);
					//odometer.setXYT(0, SquareTile*counterYF, 0); 
					LCD.drawString("ValY1: " + counterYF, 1, 5);
					LCD.drawString("ValX1: " + counterXR, 1, 6);
					counterYF++;
					
//					else {
//						//this runs at the very end when its back and turns. 
//						
//						//we add the offset back here
//						
//						//odometer.setXYT(Brick_Offset, 0, 0);
//					}
					
				}
				 else if (theta > 45 && theta < 135) { //but will this code be running during the turn? we need a delay
					// should be at 90
					 odometer.setTheta(90);
					 odometer.setX(SquareTile*counterXR);
					 //odometer.setXYT(SquareTile*counterXR, SquareTile*counterYF, 90); //the y should be constant from the previous case
					 //so we want to get y and store it here
					 //based on counter 1 we know how many lines it went through in y 
						LCD.drawString("ValY2: " + counterYF, 1, 5);
						LCD.drawString("ValX2: " + counterXR, 1, 6);
					 counterXR++; 
				} else if (theta > 135 && theta < 225) {
					// should be at 180
					//its coming back
					
					counterYF--;
					//now the X should be constant based on counter 2 we know how many x lines it passed
					odometer.setTheta(180);
					odometer.setY(SquareTile*counterYF);
					//odometer.setXYT(SquareTile*counterXR, SquareTile*counterYF, 180); //y should be changing 
					//the lines are reducing
					LCD.drawString("ValY3: " + counterYF, 1, 5);
					LCD.drawString("ValX3: " + counterXR, 1, 6);
					 
				} else if (theta > 225 && theta < 315) {
					// should be at 270
					//now its coming back to original position
					counterXR--;
					// x should be reducing and y should be 0 
					odometer.setTheta(270);
					odometer.setX(SquareTile*counterXR);
					//odometer.setXYT(SquareTile*counterXR, 0, 270);
					LCD.drawString("ValY4: " + counterYF, 1, 5);
					LCD.drawString("ValX4: " + counterXR, 1, 6);
					
				}
			}
			//there's an offset at each turn, approximately equal to an entire square. 
			
		/*Another way of doing this is hardcoded to a 3x3 grid: 
			//
			// //what turn is the robot doing?
			// switch (NoOfLines){
			//
			// case 1:
			// odometer.setXYT(0,0,0); //we have to measure offset and record it
			// case 2:
			// odometer.setY(SquareTile);
			// case 3:
			// odometer.setY(2*SquareTile);
			// case 4:
			// odometer.setTheta(90); //it turns right
			// odometer.setX(0); //there is an offset again in actual measurement
			// case 5:
			// odometer.setX(SquareTile);
			// case 6:
			// odometer.setX(2*SquareTile);
			// case 7:
			// odometer.setTheta(180);
			// odometer.setY(2*SquareTile);
			// case 8:
			// odometer.setY(SquareTile);
			// case 9:
			// odometer.setY(0);
			// case 10:
			// odometer.setTheta(270);
			// odometer.setX(2*SquareTile);
			// case 11:
			// odometer.setX(SquareTile);
			// case 12:
			// odometer.setXYT(Brick_Offset, 0 ,0);
			// //add the offset back.
			// }
			//*/

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}

}