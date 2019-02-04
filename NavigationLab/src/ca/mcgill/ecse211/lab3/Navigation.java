package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class provides the navigation functionality of the EV3. 
 * @author Preyansh & Maxime
 *
 */
public class Navigation extends Thread {

	private EV3LargeRegulatedMotor leftMotor; 
	private EV3LargeRegulatedMotor rightMotor; 
	private final double TRACK; 
	private final double WHEEL_RAD; 
	private double SQUARE_TILE = 30.48; 
	private Odometer odometer;  
	private OdometerData odoData; 
	
private double[][]
  	  waypoints;

    private static final TextLCD lcd = LocalEV3.get().getTextLCD(); 

	int ROTATE_SPEED = 150; 

	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD, double[][] waypoints) throws OdometerExceptions{
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor; 
		this.TRACK = TRACK; 
		this.WHEEL_RAD = WHEEL_RAD; 
		this.odometer = Odometer.getOdometer();
	    odoData = OdometerData.getOdometerData();
	    odoData.setXYT(0 , 0 , 0);
	    this.waypoints = waypoints;
	   
	}
	
	//Variables
	double deltaX; 
	double deltaY; 
	double deltaT;
	
	double currentX;
	double currentY;
	double currentT; 
	double dist; 
	double dTheta; 
	
	public void run() {
		//Threaded method, main code goes here. 
		
	//	selectMap(waypoints, SQUARE_TILE); //allows the user to select a map to navigate
		
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			
		}
		
		for (int i = 0; i < waypoints.length; i++) { 
			travelTo(waypoints[i][0], waypoints[i][1]);
		}
	}
	
	void travelTo(double x, double y) {
		//TODO: This method should cause robot to go to (x,y) coordinates entered. 
		
		double position[] = new double[3]; 
		position = odometer.getXYT(); 
		currentX = position[0];
		currentY = position[1];
		currentT = position[2];
		
		deltaX = x - currentX; 
		deltaY = y - currentY; 
		dist = Math.sqrt(deltaX * deltaX + deltaY*deltaY); 
		
		//Three cases based on deltaX, deltaY
		
		if(deltaY >= 0) {
			//the robot has to move in positive Y direction to where it is facing
			
			//Y is positive, x could be anything because Math.atan calculates from pi/2 to -pi/2
			//i.e. when Y is positive (quadrants 1 and 2) 
			
			dTheta = Math.atan(deltaX/deltaY); //tan is opposite divided by adjacent
		
			
		}
		else if(deltaY <=0 && deltaX >= 0) {
			//y is negative, x is positive
			//quadrant 4
			
			dTheta = Math.atan(deltaX/deltaY) + Math.PI; //minimal angle calculation
			//Math.PI is factored in because the range of atan is from -pi/2 to pi/2
			//tan(-a) = -tan(a) which is why Math.PI is added not subtracted
		}
		else {
			//y is negative, x is negative.
			//quadrant 3
			
			dTheta = Math.atan(deltaX/deltaY) - Math.PI; 
		}
		
		double thetaDiff = (Math.toDegrees(dTheta)-currentT);
		//The robot is not always facing 0, so needs to find the difference it needs to turn
		turnTo(thetaDiff); 
		
		leftMotor.setSpeed(250);
		rightMotor.setSpeed(250);
		leftMotor.rotate(convertDistance(WHEEL_RAD, dist), true);
	    rightMotor.rotate(convertDistance(WHEEL_RAD, dist), false); 
		//convertDistance borrowed from SquareDriver
	    
	    
	
	}
	
	void turnTo(double theta) {
		//TODO: This method should cause the robot to turn to theta
		
		
		if(theta>180) {
			theta=360-theta;
			leftMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
		    rightMotor.setSpeed(ROTATE_SPEED);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);

			}
		
		else if(theta<-180) {
			
			theta=360+theta;
			leftMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.setSpeed(ROTATE_SPEED);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
			}
		
		else {	
			
			leftMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.setSpeed(ROTATE_SPEED);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
	}
		

	
	boolean isNavigating() {
		//boolean indicates whether travelTo and turnTo are running
		boolean result = false; 
		if(leftMotor.isMoving() && rightMotor.isMoving()) {
			result = true; 
		}
		
		return result; 
	}
	
		
	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	  private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }
	  
	  

	  private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
	  
	}
