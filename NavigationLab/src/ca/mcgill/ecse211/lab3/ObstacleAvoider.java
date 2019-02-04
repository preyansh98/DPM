package ca.mcgill.ecse211.lab3;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;

public class ObstacleAvoider extends Thread {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usDistance ;
	private final double TRACK;
	private final double WHEEL_RAD;
	
	public static final int MOTOR_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	double currentX, currentY, currentT;
	double deltaX, deltaY, dTheta;
	double dist;
	int counter = 0;
	private Odometer odometer;
	private OdometerData odoData;
	private Navigation usNavigator; 
	private double[][]  waypoints = new double[][]
		{{0*30.48,2*30.48}, 
		{1*30.48,1*30.48},
		{2*30.48,2*30.48},
		{2*30.48,1*30.48},
		{1*30.48,0*30.48}};
		// Map1
	
//        {{1*30.48, 1*30.48},
//		{0*30.48, 2*30.48},
//		{2*30.48, 2*30.48},
//		{2*30.48,1*30.48},
//		{1*30.48, 0*30.48}};
		// Map2
								
//	 {{1*30.48, 0*30.48},
//	 {2*30.48, 1*30.48},
//	 {2*30.48, 2*30.48},
//	 {0*30.48,2*30.48},
//	 {1*30.48, 1*30.48}};
	// Map3

//		{{0*30.48, 1*30.48},
//		{1*30.48, 2*30.48},
//		{1*30.48, 0*30.48},
//		{2*30.48, 1*30.48},
//		{2*30.48, 2*30.48}};
		// Map4
		

		public ObstacleAvoider(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
				final double TRACK, final double WHEEL_RAD, double[][] waypoints) throws OdometerExceptions {

			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;

			this.odometer = Odometer.getOdometer();
			odoData = OdometerData.getOdometerData();
			odoData.setXYT(0,0,0);
			this.TRACK = TRACK;
			this.WHEEL_RAD = WHEEL_RAD;
			SensorModes usSensor = new EV3UltrasonicSensor(usPort);
			usDistance = usSensor.getMode("Distance"); 
			this.usData = new float[usDistance.sampleSize()]; 
			
			this.waypoints = waypoints; 
		}

		// run method 
		public void run() {
			
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
				motor.stop();
				
				motor.setAcceleration(300);  
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
			
			currentX = odometer.getXYT()[0];
			currentY = odometer.getXYT()[1];
			currentT = odometer.getXYT()[2];

		deltaX = x- currentX;
		deltaY = y - currentY;
		
		dist = Math.sqrt(deltaX*deltaX+deltaY*deltaY);
		
		if(deltaY>=0) {
			dTheta=Math.atan(deltaX/deltaY);
		}
		else if(deltaY<=0&&deltaX>=0) {
			dTheta=Math.atan(deltaX/deltaY)+Math.PI;
		}
		else {
			dTheta=Math.atan(deltaX/deltaY)-Math.PI;
		}

			//Finds the difference in theta and from it turn the robot in the direction with the shortest Theta 
			double thetaDifference = (dTheta*180/Math.PI-currentX); 
			turnTo(thetaDifference); 

			// moves forward for the required distance
			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			leftMotor.rotate(convertDistance(WHEEL_RAD, dist), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, dist), true);

			while(isNavigating()) { 
				usDistance.fetchSample(usData,0);
				float distance = usData[0]*100;
				
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