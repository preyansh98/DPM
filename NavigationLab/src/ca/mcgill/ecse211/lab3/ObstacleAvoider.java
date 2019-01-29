package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.ultrasonic.UltrasonicController;

/**
 * A threaded class is needed that ensures the obstacles are turned away from using the Ultrasonic Sensor
 * @author Preyansh
 *
 */
public class ObstacleAvoider extends Thread {
	
	UltrasonicController us;
	
	public void run() {
		//Threaded code 
	}
	
	public ObstacleAvoider(UltrasonicController us) {
		this.us = us; //initializes an ultrasonic controller in this class
	}
	
		
	
}
