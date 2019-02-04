package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;


/**
 * This is the main class Lab3 that initializes all sensors and passes arguments to constructors of
 * other classes. 
 * @author Preyansh & Maxime
 *
 */
public class Lab3 {
	
	  // Motor Objects, and Robot related parameters
	  private static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	  private static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	  private static final TextLCD lcd = LocalEV3.get().getTextLCD(); 
	  public static final double WHEEL_RAD = 2.19; //2.19 set as the radius of the wheel
	  public static final double TRACK = 10.7; //The track value refers to distance between the two wheels.



	  private static double SQUARE_TILE = 30.48; 
	  
	  //the waypoints are fixed and updated before each map trial
	  private static double[][] 
	    	  waypoints = new double[][] 	   
			    	   {{2*SQUARE_TILE, 1*SQUARE_TILE},
			  			{1*SQUARE_TILE, 1*SQUARE_TILE},
						{1*SQUARE_TILE, 2*SQUARE_TILE},
						{2*SQUARE_TILE, 0*SQUARE_TILE},
						};  ; 
						
	
	/**
	 * The main method initializes all the threads and depending on user input on 
	 * button pressed, runs the required navigation thread. 				
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions{
		   int buttonChoice;
		    // Odometer related objects
		    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		    Display odometryDisplay = new Display(lcd); // No need to change
		    
		    //Objects of the navigation and obstacle avoider class are created here
		    //We pass in the motors, track, wheel_rad and the waypoints for the map chosen
			Navigation navigator = new Navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD, waypoints);   
		  	ObstacleAvoider obstacleAvoider = new ObstacleAvoider(leftMotor, rightMotor, TRACK, WHEEL_RAD, waypoints);
		    do {
		      // clear the displays
		      lcd.clear();

	          lcd.drawString("< Left  | Right >", 0, 0);
	          lcd.drawString("  No    | with   ", 0, 1);
	          lcd.drawString("obstacle|obstacle", 0, 2);
	          lcd.drawString("        |   ", 0, 3);
	          lcd.drawString("        |        ", 0, 4);

	        buttonChoice = Button.waitForAnyPress();
	      }
    
	    while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);		      
		
		    //This method starts the threads for the odometer and display
		    initThreads(odometer, odometryDisplay); 
		    
		    if(buttonChoice == Button.ID_LEFT) {	
		    	//runs the navigation class
		    	navigator.run();
		    }
		    else if(buttonChoice == Button.ID_RIGHT) {
		    	//runs the obstacle avoider class
				obstacleAvoider.run();
		    }

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