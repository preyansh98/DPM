package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.Port;//just added



/**
 * This is the main class Lab3 that initializes all sensors and passes arguments to constructors of
 * other classes. 
 * @author Preyansh & Maxime
 *
 */
public class Lab4 {
	
	  // Motor Objects, and Robot related parameters
	  private static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	  private static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	  private static final TextLCD lcd = LocalEV3.get().getTextLCD(); 
	  public static final double WHEEL_RAD = 2.19; //2.19 set as the radius of the wheel
	  public static final double TRACK = 10.7; //The track value refers to distance between the two wheels.
	  private static final Port sensorPortUS = LocalEV3.get().getPort("S1");//just added
	 
	
	/**
	 * The main method initializes all the threads and depending on user input on 
	 * button pressed, runs the required navigation thread. 				
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions{
		//Setting up the sensor
		@SuppressWarnings("resource")	
		SensorModes usSensor = new EV3UltrasonicSensor(sensorPortUS);
		//SensorModes usSensor = new EV3UltrasonicSensor(SensorPort.S1);		
		SampleProvider usDistance = usSensor.getMode("Distance");	
		float [] usData = new float[usDistance.sampleSize()];
			


		
		   int buttonChoice;
		    // Odometer related objects
		    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		    Display odometryDisplay = new Display(lcd); // No need to change
		    
		    //Objects of the navigation and obstacle avoider class are created here
		    //We pass in the motors, track, wheel_rad and the waypoints for the map chosen
			
		  
	    	LightLocalizer lsLocalizer = new LightLocalizer(leftMotor, rightMotor, odometer, TRACK, WHEEL_RAD); 
			
			
			do {
		      // clear the displays
		      lcd.clear();

	          lcd.drawString("< Left  | Right >", 0, 0);
	          lcd.drawString("Falling | Rising ", 0, 1);
	          lcd.drawString(" edge   |  edge  ", 0, 2);
	          lcd.drawString("        |        ", 0, 3);
	          lcd.drawString("        |        ", 0, 4);

	        buttonChoice = Button.waitForAnyPress();
	      }
    
	    while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);		      
		
		    //This method starts the threads for the odometer and display
		   UltrasonicLocalizer usLocalizer = null; 
		    
		    if(buttonChoice == Button.ID_LEFT) {	
		    	//starts the ultrasonic localizer class with falling edge
		    	usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer,
		    			TRACK, WHEEL_RAD, UltrasonicLocalizer.edgeType.FallingEdge, usDistance);	 
		    }
		    else if(buttonChoice == Button.ID_RIGHT) {
		    	//starts the ultrasonic localizer class with rising edge
		    	usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer,
		    			TRACK, WHEEL_RAD, UltrasonicLocalizer.edgeType.FallingEdge, usDistance);
		    }
			
		    initThreads(odometer, odometryDisplay);
		    usLocalizer.mainMethod(); 


			while (Button.waitForAnyPress() != Button.ID_RIGHT)
				;

			// implement the light sensor localization
			lsLocalizer.mainMethod();
			
			if(Button.waitForAnyPress() == Button.ID_ESCAPE) {
				System.exit(0);
			}
			
			//add lightsensor calling here
			
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		    System.exit(0);
		  
	}
	
	/**
	 * Simple method that initializes the threads for the odometer and odometer display
	 * @param odometer
	 * @param odometryDisplay
	 */
	private static void initThreads(Odometer odometer, Display odometryDisplay) {
		
	    Thread odoThread = new Thread(odometer);
	      odoThread.start();
	      Thread odoDisplayThread = new Thread(odometryDisplay);
	      odoDisplayThread.start();
	   
	}
	
	}