package ca.mcgill.ecse211.lab4;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;

/**
 * This class initializes and runs the light localizer allowing the robot to turn to the origin correctly. 
 * @author Preyansh
 * @author Maxime
 *
 */
public class LightLocalizer {
	
	//The Sensor Offset is the recorded distance between the centre of rotation (between the wheels) and the light sensor. 
	private final double SENSOR_OFFSET = 13; 

	//Speeds for the motors
	private static int ROTATE_SPEED = 50;
	private static int MOTOR_SPEED = 100; 
	
	//Light Sensor specific variables
	private static final double lineThreshold = 185; 
	private int lineCounter = 0; 
	double[] ThetaArray = new double[4];
	
	//Odometer
	private Odometer odometer; 
	
	//Motors
	private EV3LargeRegulatedMotor leftMotor, rightMotor; 
	
	//Initializing light sensor
	private static final Port lsPort = LocalEV3.get().getPort("S4"); 
	EV3ColorSensor colSensor = new EV3ColorSensor(lsPort);
	SampleProvider lsValue; 
	private float[] lsData; 
	
	//other global variables
	private double TRACK = 0; 
	private double WHEEL_RAD = 0;  
	
	/**
	 * The constructor takes in references to the motors, odometer, and TRACK/WHEEL_RAD values from main class
	 * The color sensor is also initialized here to red mode, and an array is initialized to store samples. 
	 * @param leftMotor
	 * @param rightMotor
	 * @param odometer
	 * @param TRACK
	 * @param WHEEL_RAD
	 */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			Odometer odometer, double TRACK, double WHEEL_RAD) {
		this.odometer = odometer; 
		this.leftMotor = leftMotor; 
		this.rightMotor = rightMotor; 
		
		this.TRACK = TRACK; 
		this.WHEEL_RAD = WHEEL_RAD; 
		colSensor.setCurrentMode("Red");
		this.lsValue = colSensor.getMode("Red");
		this.lsData = new float[lsValue.sampleSize()];
	}
	
	/**
	 * This is the main method that is called upon in the main. 
	 * This method makes the robot turn to the origin, apply a correction algorithm, and stop at origin in correct angle finally. 
	 */
	public void mainMethod() {
		
		//First, we want it to turn to 45 degrees, i.e. where the origin is
		turnTo(45); 
		

		//Move till the sensor detects black line close to origin. 
		while(getIntensity() > lineThreshold) {
			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			leftMotor.forward();
			rightMotor.forward();
			LCD.drawString("value: " + getIntensity(), 0, 5);
			//Moves forward until a black line is found, when a line is found, it breaks out of this loop. 
			if(getIntensity() < lineThreshold) {
				break;
			}
		}
		  leftMotor.stop(true);
		  rightMotor.stop();
		  Sound.beep();
		  
		//move back by the Sensor Offset value
		leftMotor.setSpeed(MOTOR_SPEED);
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_OFFSET), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_OFFSET), false);
		
		//Now advance forward until you read a line
		
		while(lineCounter < 4) {
			//While it hasn't detected all four lines, it will continuosly turn clockwise 
			turnClockwise(); 

			//In this loop when it detects a line, it will store the theta values in an array 
			if(getIntensity() < lineThreshold) {
				ThetaArray[lineCounter] = odometer.getXYT()[2];
				Sound.setVolume(10);
				  Sound.systemSound(false, 4);
				  lineCounter++;
			}			
		}

		/*We now have the theta values stored, makes calculations to move to origin now*/
		  double dX, dY, anglex, angley;

		  // Get our location from origin using the calculated angles
		  angley = ThetaArray[3] - ThetaArray[1];
		  anglex = ThetaArray[2] - ThetaArray[0];

		  //Based on the angles calculated, calculates the current x and y position
		  dX = -SENSOR_OFFSET*Math.cos(Math.toRadians(angley/2)); 
		  dY = -SENSOR_OFFSET*Math.cos(Math.toRadians(anglex/2));
		  		  
		  //Sets current position in odometer to the x and y calculated
		  odometer.setXYT(dX, dY, odometer.getXYT()[2]);
		  //Moves to the origin relative to this
		  this.travelTo(0.0, 0.0);
		  do {
    	  LCD.drawString("X: " +dX, 0, 5);
    	  LCD.drawString("Y: " + dY, 0, 6);
		  }
		  while (isNavigating()); 
	      
		  //Now make it turn to 0. 
		  Sound.setVolume(20);
		  Sound.systemSound(false, 2);   
		  
		  //Based on the current angle reported from the odometer, will move to correct the angle, i.e. turn to 0. 
		  double currentangle = odometer.getXYT()[2]; 
		  turnTo(-currentangle);

		  
	}
	
	/**
	 * This method makes the robot turn clockwise.
	 */
	private void turnClockwise() {
		leftMotor.forward();
		rightMotor.backward();		
	}
	
	/**
	 * This method is borrowed from the Navigation class. 
	 * It uses the current position of the robot from the odometer, and calculates how far it needs to move based on that
	 * 
	 * Based on in which quadrant it needs to move, the angle is calculated differently to ensure minimal angle
	 * @param x - the x co-ordinate of the waypoint to travel to
	 * @param y - the y co-ordinate of the waypoint to travel to
	 */
	void travelTo(double x, double y) {

		double currentX, currentY, currentT, deltaX, deltaY, dist; 
		double dTheta = 0; 
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
	 * This method is also borrowed from Navigation class. 
	 * It makes the robot turn to a specific angle entered. 
	 * @param theta
	 */
	private void turnTo(double theta) {		
		
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
	 * This boolean method check whether motors are moving (i.e. robot is navigating)
	 * @return a boolean that returns true if the motors are moving
	 */
	private boolean isNavigating() {
		//boolean indicates whether travelTo and turnTo are running
		boolean result = false; 
		if(leftMotor.isMoving() && rightMotor.isMoving()) {
			result = true; 
		}
		
		return result; 
	}
	

	/**
	 * This is a getter for the Intensity value from the light sensor. 
	 * The sample is fetched, stored, and returned to the class
	 * @return a double value of the light sensor reading. 
	 */
	private double getIntensity() {
		colSensor.fetchSample(lsData, 0);
		double lsValue = lsData[0] * 1000; 
		return lsValue;
		
	}
	
    /**
     * This method allows the conversion of a distance to the total rotation of each wheel need to
     * cover that distance.
     * Borrowed from SquareDriver class
     *
     * @param radius
     * @param distance
     * @return an integer tacho count
     */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }
    
    /**
     * This method takes in the radius, width, and angle and converts the angle to a 
     * tacho count to rotate the wheels. 
     * @param radius
     * @param width
     * @param angle
     * @return an integer tacho count of how much the motors should rotate to get this angle
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }

}
