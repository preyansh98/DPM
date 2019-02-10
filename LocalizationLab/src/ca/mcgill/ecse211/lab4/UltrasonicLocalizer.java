package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer extends Thread{
	
	//Setting up motors
		private EV3LargeRegulatedMotor leftMotor;
		private EV3LargeRegulatedMotor rightMotor;
		
		//Ultrasonic sensor initializers
		private float[] usData;
		private SampleProvider usSamples;
		
		
		private final double TRACK = 0;
		private final double WHEEL_RAD = 0;
		public static final int MOTOR_SPEED = 250;
		private static final int ROTATE_SPEED = 150;
		
		//Other global variables

		
		//Odometer initializers
		private Odometer odometer;
		private OdometerData odoData;
		
		//Enum created to distinguish between whether to start with rising edge or falling edge
		public enum edgeType {
			FallingEdge, RisingEdge; 
		}
		private edgeType type; 
		
		public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD, edgeType type) throws OdometerExceptions {
			
			//Inheriting references to the motors first
			this.leftMotor = leftMotor; 
			this.rightMotor = rightMotor; 
			
			//Setting up odometer to 0,0,0
			this.odometer = Odometer.getOdometer(); 
			this.odoData = Odometer.getOdometerData(); 
			odoData.setXYT(0, 0, 0); //the orientation is set according to where robot starts
			
			//Setting up the ultrasonic sensor
			SensorModes usSensor = new EV3UltrasonicSensor(SensorPort.S1);
			usSamples = usSensor.getMode("Distance"); 
			this.usData = new float[usSamples.sampleSize()]; 
			
			//Setting up whether falling edge or rising edge
			this.type = type; 
			
		}
		
		public void run() {
			//The main threaded method.
			
			if(type == edgeType.FallingEdge) {
				fallingEdge();
			}
			else if(type == edgeType.RisingEdge){
				risingEdge(); 
			}
			else {
				throw new IllegalArgumentException(); 
			}
			
		}
		
		public void fallingEdge() {
			
		}
		
		public void risingEdge() {
				
		}

	
}
