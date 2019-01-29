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
	  public static final double TRACK = 11; //The track value refers to distance between the two wheels.

	   static EV3ColorSensor colSensor = new EV3ColorSensor(SensorPort.S3); //Color Sensor initialized 
	
	public static void main(String[] args) throws OdometerExceptions{
		   int buttonChoice;
		    // Odometer related objects
		    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
		    OdometryCorrection odometryCorrection = new OdometryCorrection(Odometer.getOdometer(), colSensor); // TODO Complete
		                                                                      // implementation
		    Display odometryDisplay = new Display(lcd); // No need to change


		    do {
		      // clear the display
		      lcd.clear();

		      // ask the user whether the motors should drive in a square or float
		      lcd.drawString("< Left | Right >", 0, 0);
		      lcd.drawString("       |        ", 0, 1);
		      lcd.drawString(" Float | Drive  ", 0, 2);
		      lcd.drawString("motors | in a   ", 0, 3);
		      lcd.drawString("       | square ", 0, 4);

		      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		    if (buttonChoice == Button.ID_LEFT) {
		      // Float the motors
		      leftMotor.forward();
		      leftMotor.flt();
		      rightMotor.forward();
		      rightMotor.flt();

		      // Display changes in position as wheels are (manually) moved
		      
		      Thread odoThread = new Thread(odometer);
		      odoThread.start();
		      Thread odoDisplayThread = new Thread(odometryDisplay);
		      odoDisplayThread.start();

		    } else {
		      // clear the display
		      lcd.clear();

		      // ask the user whether odometery correction should be run or not
		      lcd.drawString("< Left | Right >", 0, 0);
		      lcd.drawString("  No   | with   ", 0, 1);
		      lcd.drawString(" corr- | corr-  ", 0, 2);
		      lcd.drawString(" ection| ection ", 0, 3);
		      lcd.drawString("       |        ", 0, 4);

		      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		      // Start odometer and display threads
		      Thread odoThread = new Thread(odometer);
		      odoThread.start();
		      Thread odoDisplayThread = new Thread(odometryDisplay);
		      odoDisplayThread.start();

		      // Start correction if right button was pressed
		      if (buttonChoice == Button.ID_RIGHT) {
		        Thread odoCorrectionThread = new Thread(odometryCorrection);
		        odoCorrectionThread.start();
		      }

		      // spawn a new Thread to avoid SquareDriver.drive() from blocking
		      (new Thread() {
		        public void run() {
		          
		        }
		      }).start();
		    }

		    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		    System.exit(0);
		  }

	}








 