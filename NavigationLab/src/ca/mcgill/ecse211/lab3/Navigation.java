package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
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
  	  waypoints; //initializes a double array of waypoints 

    private static final TextLCD lcd = LocalEV3.get().getTextLCD(); //

	int ROTATE_SPEED = 150; 
/**
 * this constructor sets up the leftMotor, rightMotor, track between the wheels, the radius of the wheels and the odometer
 * @param leftMotor
 * @param rightMotor
 * @param TRACK
 * @param WHEEL_RAD
 * @param waypoints
 * @throws OdometerExceptions
 */
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
	
	//initialization of Variables
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
		
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			
		}
		
		//This is the main for loop that makes the robot travel to the waypoints, 
		//row by row
		for (int i = 0; i < waypoints.length; i++) { 
			travelTo(waypoints[i][0], waypoints[i][1]);
		}
	}
	/**
	 * This method allows the robot to travel to each waypoints based on its x and y coordinates
	 * It uses the current position of the robot from the odometer, and calculates how far it needs to move based on that
	 * 
	 * Based on in which quadrant it needs to move, the angle is calculated differently to ensure minimal angle
	 * @param x - the x co-ordinate of the waypoint to travel to
	 * @param y - the y co-ordinate of thew waypoint to travel to
	 */
	void travelTo(double x, double y) {
		//TODO: This method should cause robot to go to (x,y) coordinates entered. 
		
		double position[] = new double[3]; //create a array of size 3 for the odometer
		position = odometer.getXYT(); // fill the array with the XYT components from odometer
		currentX = position[0];// get x value at first position of array
		currentY = position[1];// get y value at second position of array
		currentT = position[2]; // get theta value at third position of array
		
		deltaX = x - currentX; //store difference between waypoint coordinates and odometer coordinates in a variable
		deltaY = y - currentY; 
		dist = Math.sqrt(deltaX * deltaX + deltaY*deltaY); //store shortest distance from waypoint in variable
		
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

		/**
		 * This method makes the robot turn to a specific angle entered. 
		 * @param theta
		 */
		void turnTo(double theta) {		
			
			if(theta>180) {
				
				/*
				* If the angle is more than 180, then we need the minimal angle instead, i.e. 360 - angle. 
				*/
				theta=360-theta;
				leftMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
			        rightMotor.setSpeed(ROTATE_SPEED);
				rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);

				}
			
			else if(theta<-180) {
				
				/*
				* If the angle is less than -180, similarly the difference with 360 is needed. 
				*/
				theta=360+theta;
				leftMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.setSpeed(ROTATE_SPEED);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
			
			}
			
			else {	
				
				/*
				* Angle is within the range so it is already minimal angle. 
				*/
				leftMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
				rightMotor.setSpeed(ROTATE_SPEED);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
			
			}
		}
		

	/**
	 * This method returns true if both the left motor and right motor is moving
	 * @return a boolean indicating whether robot is moving
	 */
	boolean isNavigating() {
		//boolean indicates whether travelTo and turnTo are running
		boolean result = false; 
		if(leftMotor.isMoving() && rightMotor.isMoving()) {
			result = true; 
		}
		
		return result; 
	}
	
		
	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel needed to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	  private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }
	  
	  
	  /**
	   * 
	   * @param radius
	   * @param width
	   * @param angle
	   * @return
	   */
	  private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
	  
	}
