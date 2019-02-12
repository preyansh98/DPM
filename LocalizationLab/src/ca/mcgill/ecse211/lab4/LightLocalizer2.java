package ca.mcgill.ecse211.lab4;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;

public class LightLocalizer2 {
	
	private final double SENSOR_OFFSET = 14; 
	public static int FORWARD_SPEED = 100; 
	private static final double lineThreshold = 0.3; 

	//Odometer
	private Odometer odometer; 
	
	//Motors
	private EV3LargeRegulatedMotor leftMotor, rightMotor; 
	
	//Setting up light sensor
	private static final Port lsPort = LocalEV3.get().getPort("S4"); 
	SensorModes lsSamples = new EV3ColorSensor(lsPort); 
	SampleProvider lsValue = lsSamples.getMode("Red"); 
	private float[] lsData = new float[lsValue.sampleSize()]; 
	
	
	
	
	//1. turn to the 45 degrees clockwise (i.e. face origin)
	//2. advance forward until you find a line. 
	//3. at this FIRST line, go backwards by the offset
	//4. turn 360: while loop 
		//the first line you pass in 360, log x and y
		//keep logging until you turn back to the original
		//record the x and y
	//5. move to the origin using this x and y
}
