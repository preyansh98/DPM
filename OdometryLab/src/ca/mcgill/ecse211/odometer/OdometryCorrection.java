/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class handles the corrections to apply to the Odometer by setting the XYT values to the correct values
 * based on the grid. 
 * This particular implementation uses the EV3 Color Sensor and detects light intensity to apply correction
 * at the black lines. 
 * @author Preyansh & Maxime
 *
 */
public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	private static final double LINE_INTENSITY = 85; 
	//The line intensity value was found by monitoring the light sensor readings to find a threshold below
	//which the black line intensity occurs
	private static final double SQUARE_TILE = 30.48;
	//The SQUARE_TILE is a preset value that determines the real distance between two black lines. 
	
	
	//Initializing the Color Sensor
	SampleProvider samples; //This is an abstraction by lejos.robotics that allows samples of data (light intensity values) to be collected
	private float[] sampleList; //The samples of data are stored in a float array
	private float lightPoll; //A float variable to store the value (sample) that the light sensor reads
	EV3ColorSensor colSensor; //This is the Color Sensor instance itself
	double position[] = new double[3]; // an array to hold the XYT positions from the odometer

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * The default constructor was modified to initialize the color sensor when the OdometryCorrection
	 * class is called. 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection(Odometer odometer, EV3ColorSensor colSensor) throws OdometerExceptions {
		this.odometer = odometer;
		this.colSensor = colSensor;
		colSensor.setCurrentMode("Red"); //The Red Mode is selected, since this returns the light intensity sampler. 
		this.samples = colSensor.getMode("Red"); //samples are got from the red mode that was initialized
		this.sampleList = new float[colSensor.sampleSize()]; //samples are stored in a float array that equals the size of the sample
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * This method runs by viewing whether the light intensity sample is lower than a certain threshold. 
	 * This threshold is calculated based on the black light intensity upper bound. 
	 * Then, based on the theta value, a counter system is implemented to count the number of black lines in 
	 * a certain direction. 
	 * Based on the number of black lines passed, a correction is applied each time since we know the size of the square tile. 
	 * 
	 * @throws OdometerExceptions
	 */
	   double counterYF = 0, counterXR= 0; //Initialized two counters that count number of tiles in the Y forward direction (YF) and X Right direction (XR)
	   
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();
			
			//The Color Sensor gets the sample values and stores them in the float array sampleList at [0]
			colSensor.fetchSample(sampleList, 0); 
			//Since in thread, continously updates sampleList at [0]

			//Since this entire method is threaded, it runs in a continous loop, so the sample is updated
			//to position [0] of the array every time. 
			lightPoll = sampleList[0] * 1000; 
			//The sample is normalized between 0 and 1, so we multiply it by 1000 to amplify the changes.		
			   
			if (lightPoll < LINE_INTENSITY) {
				//If the light intensity value detected is less than a threshold, then a black line is detected.
				Sound.beep(); //Added a sound to make it clear in testing whether the light sensor is working.
				position = odometer.getXYT();
				double theta = position[2]; //Storing the current theta value in a variable theta
				
			
				if (theta > 315 || theta < 45) { 
					//When the theta value is either between 315 to 360, or from 0 to 45, we know it is moving
					//in the forward direction. i.e. theta should be approximately 0. 
					odometer.setTheta(0); //A correction is applied to make theta 0.
					odometer.setY(SQUARE_TILE*counterYF); //The value of Y is set on how many square tiles are passed (i.e. counterYF)
					counterYF++;
					}
				 else if (theta > 45 && theta < 135) { 

					 odometer.setTheta(90); //Right turn is made, so theta should be 90
					 odometer.setX(SQUARE_TILE*counterXR); //The value of X is set on how many square tiles are passed (i.e counterXR)
					 counterXR++; 
					 
				} else if (theta > 135 && theta < 225) {
					counterYF--; //counter is subtracted before, because counter increments at last tile once extra
					odometer.setTheta(180); //This is the turn where the EV3 is heading back. 
					odometer.setY(SQUARE_TILE*counterYF); //Y is set according to the counter again
					 
				} else if (theta > 225 && theta < 315) {
					counterXR--; //Counter is subtracted before again, because counterXR increments at last tile extra again
					odometer.setTheta(270); //This turn is towards the starting position so should be 270
					odometer.setX(SQUARE_TILE*counterXR); //The X is set according to counter again 
				}
			}

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