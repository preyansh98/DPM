package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class provides the navigation functionality of the EV3. 
 * @author Preyansh & Maxime
 *
 */
public class Navigation extends Thread implements Runnable {

	EV3LargeRegulatedMotor leftMotor; 
	EV3LargeRegulatedMotor rightMotor; 
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		//TODO: Arguments that the constructor should take?
		//left Motor, rightMotor
		
		this.leftMotor = leftMotor; 
		this.rightMotor = rightMotor; 
	
		
	}
	public void run() {
		//Threaded method, main code goes here. 
		
		
		
		
		
	}
	
	void travelTo(double x, double y) {
		//TODO: This method should cause robot to go to (x,y) coordinates entered. 
		
	}
	
	void turnTo(double theta) {
		//TODO: This method should cause the robot to turn to theta
	}
	
	boolean isNavigating() {
		//boolean indicates whether travelTo and turnTo are running
		
		return false; 
	}
}
