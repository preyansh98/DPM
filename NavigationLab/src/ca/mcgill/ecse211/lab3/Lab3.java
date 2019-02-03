package ca.mcgill.ecse211.lab3;



import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
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

	  private static double[][] waypoints; 
	  private static double SQUARE_TILE = 30.48; 
	  
	   
	public static void main(String[] args) throws OdometerExceptions{
		   int buttonChoice;
		    // Odometer related objects
		    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		    Display odometryDisplay = new Display(lcd); // No need to change
			Navigation navigator = new Navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		    ObstacleAvoider obstacleAvoider = new ObstacleAvoider(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		    
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
		    
		    initThreads(odometer, odometryDisplay); 
		    
		    if(buttonChoice == Button.ID_LEFT) {		  
		    	navigator.run();
		    }
		    else if(buttonChoice == Button.ID_RIGHT) {
		    	obstacleAvoider.run();
		    }

		    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		    System.exit(0);
		  }

	private static void initThreads(Odometer odometer, Display odometryDisplay) {
	    Thread odoThread = new Thread(odometer);
	      odoThread.start();
	      Thread odoDisplayThread = new Thread(odometryDisplay);
	      odoDisplayThread.start();
	   
	}
	
	private static  void selectMap(double[][] waypoints, double SQUARE_TILE) {
		 // ask the user whether the motors should drive in a square or float
			int buttonChoice; 
			do {
	      lcd.drawString(" ^ Map 1 ^ ", 0, 0);
	      lcd.drawString("          |        ", 0, 1);
	      lcd.drawString(" < Map 2 | Map 3 >  ", 0, 2);
	      lcd.drawString("      ", 0, 3);
	      lcd.drawString(" Map 4 v ", 0, 4);
	      		    	
	      
	      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

	
	    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT &&
	    		buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN	);

	      if(buttonChoice == Button.ID_UP) {
	    	  //Run map 1 code 
	    	  waypoints = new double[][] 	   
	    	   {{0*SQUARE_TILE, 2*SQUARE_TILE},
	  			{1*SQUARE_TILE, 1*SQUARE_TILE},
				{2*SQUARE_TILE, 2*SQUARE_TILE},
				{2*SQUARE_TILE, 1*SQUARE_TILE},
				{1*SQUARE_TILE, 0*SQUARE_TILE}};
	      }
	      else if(buttonChoice == Button.ID_LEFT) {
	    	  //Run map 2 code 
	    	  waypoints = new double[][] 	   
			    	   {{1*SQUARE_TILE, 1*SQUARE_TILE},
			  			{0*SQUARE_TILE, 2*SQUARE_TILE},
						{2*SQUARE_TILE, 2*SQUARE_TILE},
						{2*SQUARE_TILE, 1*SQUARE_TILE},
						{1*SQUARE_TILE, 0*SQUARE_TILE}};
	      }
	      else if(buttonChoice == Button.ID_RIGHT) {
	    	  //Run map 3 code
	    	  waypoints = new double[][] 	   
			    	   {{1*SQUARE_TILE, 0*SQUARE_TILE},
			  			{2*SQUARE_TILE, 1*SQUARE_TILE},
						{2*SQUARE_TILE, 2*SQUARE_TILE},
						{0*SQUARE_TILE, 2*SQUARE_TILE},
						{1*SQUARE_TILE, 1*SQUARE_TILE}};
	      }		    	  
	      
	      else if(buttonChoice == Button.ID_DOWN) {
	    	//Run map 4 code
	    	  waypoints = new double[][] 	   
			    	   {{0*SQUARE_TILE, 1*SQUARE_TILE},
			  			{1*SQUARE_TILE, 2*SQUARE_TILE},
						{1*SQUARE_TILE, 0*SQUARE_TILE},
						{2*SQUARE_TILE, 1*SQUARE_TILE},
						{2*SQUARE_TILE, 2*SQUARE_TILE}};
	      }	 

	}

	}








 