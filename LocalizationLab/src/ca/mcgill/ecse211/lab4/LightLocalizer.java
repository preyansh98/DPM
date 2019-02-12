package ca.mcgill.ecse211.lab4;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;

public class LightLocalizer {
	
	private final double SENSOR_OFFSET = 13; 

	private static int ROTATE_SPEED = 50;
	private static int MOTOR_SPEED = 100; 
	private static final double lineThreshold = 185; 
	private int lineCounter = 0; 
	//Odometer
	private Odometer odometer; 
	
	//Motors
	private EV3LargeRegulatedMotor leftMotor, rightMotor; 
	
	//Setting up light sensor
	private static final Port lsPort = LocalEV3.get().getPort("S4"); 
	EV3ColorSensor colSensor = new EV3ColorSensor(lsPort);
	SampleProvider lsValue; 
	private float[] lsData; 
	
	//other global variables
	private double TRACK = 0; 
	private double WHEEL_RAD = 0;  
	double[] ThetaArray = new double[4];
	
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
	
	public void mainMethod() {
		//1st we want it to turn to the origin. 
		turnTo(45); 
		

		//Move till the sensor detects origin line 
		while(getIntensity() > lineThreshold) {
			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			leftMotor.forward();
			rightMotor.forward();
			LCD.drawString("value: " + getIntensity(), 0, 5);
			if(getIntensity() < lineThreshold) {
				break;
			}
		}
		  leftMotor.stop(true);
		  rightMotor.stop();
		  Sound.beep();
		  
		//move back by the offset
		leftMotor.setSpeed(MOTOR_SPEED);
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_OFFSET), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -SENSOR_OFFSET), false);
		
		//Now advance forward until you read a line
		
		while(lineCounter < 4) {
			turnClockwise(); 

			if(getIntensity() < lineThreshold) {
				ThetaArray[lineCounter] = odometer.getXYT()[2];
				Sound.setVolume(10);
				  Sound.systemSound(false, 4);
				  lineCounter++;
			}			
		}

		/**********************************************************/
		  double dX, dY, anglex, angley, deltathetaY;

		  // Get our location from origin using the calculated angles
		  angley = ThetaArray[3] - ThetaArray[1];
		  anglex = ThetaArray[2] - ThetaArray[0];

		  dX = -SENSOR_OFFSET*Math.cos(Math.toRadians(angley/2)); 
		  dY = -SENSOR_OFFSET*Math.cos(Math.toRadians(anglex/2));
		  
		  deltathetaY = (Math.PI / 2.0) - ThetaArray[3] + Math.PI + (angley / 2.0);
		  
		  odometer.setXYT(dX, dY, odometer.getXYT()[2]);
		  this.travelTo(0.0, 0.0);
		  do {
    	  LCD.drawString("X: " +dX, 0, 5);
    	  LCD.drawString("Y: " + dY, 0, 6);
		  }
		  while (isNavigating()); 
	      
		  //Now make it turn to 0. 
		  Sound.setVolume(20);
		  Sound.systemSound(false, 2);   
		  
		  double currentangle = odometer.getXYT()[2]; 
		  turnTo(-currentangle);

		  
	}
	
	private void turnClockwise() {
		leftMotor.forward();
		rightMotor.backward();		
	}
	
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
	boolean isNavigating() {
		//boolean indicates whether travelTo and turnTo are running
		boolean result = false; 
		if(leftMotor.isMoving() && rightMotor.isMoving()) {
			result = true; 
		}
		
		return result; 
	}
	

	
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

	  private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }

	  

}
