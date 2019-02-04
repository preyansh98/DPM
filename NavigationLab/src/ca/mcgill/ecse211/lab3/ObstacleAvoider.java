package ca.mcgill.ecse211.lab3;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class ObstacleAvoider extends Thread {

	//Setting up motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	//Ultrasonic sensor initializers
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usSamples;
	
	
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final int MOTOR_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	//Other global variables
	double currentX, currentY, currentT;
	double deltaX, deltaY, dTheta;
	double dist;
	int counter = 0;
	
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
			SensorModes usSensor = new EV3UltrasonicSensor(usPort);
			usSamples = usSensor.getMode("Distance"); 
			this.usData = new float[usSamples.sampleSize()]; 
			
			this.waypoints = waypoints; 
		}

		// run method 
		public void run() {
			
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
				motor.stop();
				motor.setAcceleration(300); 
				//we set the acceleration much lower so we can read more sammples
			}
			
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				
			}
			while(counter < waypoints.length) { 
				travelTo(waypoints[counter][0], waypoints[counter][1]);
				counter++;
			}
		}

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
		    rightMotor.rotate(convertDistance(WHEEL_RAD, dist), false); 
			//convertDistance borrowed from SquareDriver

			while(isNavigating()) { 
				usSamples.fetchSample(usData,0);
				float distance = usData[0]*100; //we multiplied the sample by a 100 to increase its prominence
												//since it is normalized between 0 and 1
				
				
				if(distance<= 12) {
								
					if(odometer.getXYT()[0]<2.4*30.48 && odometer.getXYT()[0] >1.3*30.48
							&& odometer.getXYT()[1]<2.5*30.48 && odometer.getXYT()[1]>1.6*30.48 ){
						
						leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);   
						rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
						leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
						leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
						rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
						leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
					}
					
					else {
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);   
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
					}
					counter--;
				}
			}
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