package ca.mcgill.ecse211.lab3;

import lejos.hardware.sensor.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;


/**
 * This class has the same functionality as the Navigation class, but implements
 * the Navigation class travelTo method differently. 
 * 
 * @author Preyansh & Maxime
 *
 */
public class ObstacleAvoider extends Thread {

	//Setting up motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	//Ultrasonic sensor initializers
	private float[] usData;
	private SampleProvider usSamples;
	
	
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final int MOTOR_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	//Other global variables
	private double currentX, currentY, currentT;
	private double deltaX, deltaY, dTheta;
	private double dist;
	private int counter = 0;
	
	//Odometer initializers
	private Odometer odometer;
	private OdometerData odoData;
	private double[][]  waypoints;
		
	public ObstacleAvoider(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
				final double TRACK, final double WHEEL_RAD, double[][] waypoints) throws OdometerExceptions {

			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;

			//Setting up Odometer to initial 0,0,0
			this.odometer = Odometer.getOdometer();
			odoData = OdometerData.getOdometerData();
			odoData.setXYT(0,0,0);
			
			this.TRACK = TRACK;
			this.WHEEL_RAD = WHEEL_RAD;
			
			//Setting up ultrasonic sensor in distance mode to collect samples 
			SensorModes usSensor = new EV3UltrasonicSensor(SensorPort.S1);
			usSamples = usSensor.getMode("Distance"); 
			this.usData = new float[usSamples.sampleSize()]; 
			
			this.waypoints = waypoints; 
		}

		// run method 
		public void run() {
			
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
				motor.stop();
				motor.setAcceleration(300); 
				//The acceleration is significantly lowered to let the robot read more samples
			}
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				
			}
			
		/*An extra counter is created that indicates which waypoint 
		   the robot has to travel to next. */
			while(counter < waypoints.length) { 
				travelTo(waypoints[counter][0], waypoints[counter][1]);
				counter++;
			}
		}
		
		/**
		 * This method allows the robot to travel to each waypoints based on its x and y coordinates
		 * It uses the current position of the robot from the odometer, and calculates how far it needs to move based on that
		 * 
		 * Based on in which quadrant it needs to move, the angle is calculated differently to ensure minimal angle. 
		 * 
		 * It differs from Navigation class method by adding the case where the ultrasonic sensor detects an obstacle nearby
		 * In this case it runs a sequence to avoid the obstacle. 
		 * 
		 * @param x - the x co-ordinate of the waypoint to travel to
		 * @param y - the y co-ordinate of thew waypoint to travel to
		 */
		void travelTo(double x, double y) {
			
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
		    rightMotor.rotate(convertDistance(WHEEL_RAD, dist), true); 
			//convertDistance borrowed from SquareDriver

			while(isNavigating()) { 
				usSamples.fetchSample(usData,0);
				float distance = usData[0]*100; //we multiplied the sample by a 100 to increase its prominence
												//since it is normalized between 0 and 1
				
				
				if(distance<= 12) {
					//if the distance is below a certain threshold, i.e. obstacle is here
					//run this code
					
					if(odometer.getXYT()[0] > 1.1*30.48 && odometer.getXYT()[0]<2.2*30.48
							&& odometer.getXYT()[1]<2.2*30.48 && odometer.getXYT()[1]>1.7*30.48 ){ 
						// if robot is at the top right of the map, run this code
						
						//it moves a fixed pattern to avoid the block
						leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);   // makes sharp 90 degree turn first to avoid obstacle
						rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
						leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true); 
						rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
						leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
						rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
						leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
					}
					
					else {//if robot is not at top right of the map run this code
						//still moves a fixed pattern to avoid the block
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);   
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
					}
					//the counter is subtracted here so at the end of this sequence
					//it would go back to the waypoint it initially was on path to
					counter--;
				}

			
			}
		}

		/**
		 * This method makes the robot turn to a specific angle entered. 
		 * @param theta
		 */
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

		/**
		 * This method returns a boolean as true if either motor is moving. 
		 * i.e. then the robot is Navigating
		 * @return true if any of the two motors are moving
		 */
		boolean isNavigating() {
			if((leftMotor.isMoving() || rightMotor.isMoving()))
				return true;
			else 
				return false;

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
