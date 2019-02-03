package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.ultrasonic.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;

/**
 * A threaded class is needed that ensures the obstacles are turned away from using the Ultrasonic Sensor
 * @author Preyansh
 *
 */
public class ObstacleAvoider extends Thread {
	
	UltrasonicController us;
	private EV3LargeRegulatedMotor leftMotor, rightMotor; 
	private final double TRACK;
	private final double WHEEL_RAD; 
	
	public void run() {
		//Threaded code 
	}
	
	public ObstacleAvoider(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK, double WHEEL_RAD) {
		this.leftMotor = leftMotor; 
		this.rightMotor = rightMotor; 
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD; 
	}
	
		
	
}
