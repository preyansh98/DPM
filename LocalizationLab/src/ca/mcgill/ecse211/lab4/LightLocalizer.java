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

public class LightLocalizer {
	
	private final double f = 14; // distance separating the light sensor from the center of the wheels
	public static int forwardSpeed = 100;
	private static final double blackValue = 0.3;
	Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	final TextLCD display = LocalEV3.get().getTextLCD();
	// light sensor setup
	private static final Port lightPort = LocalEV3.get().getPort("S4");
	SensorModes lightSensor = new EV3ColorSensor(lightPort);
	//Samples for light sensor
	SampleProvider lightValue = lightSensor.getMode("Red"); 
	float[] lsData = new float[lightValue.sampleSize()]; 
	
	
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
	Odometer odometer) {
	this.odometer = odometer;
	this.leftMotor = leftMotor;
	this.rightMotor = rightMotor;
	}
	
	
	/**
	 * lightLocalization uses the light sensor to get to the origin, using the lines detected and the distances measured
	 *  from each line detection. 
	 **/
	
	public void lightLocalize(){
	
	//setting speed and moving forward - waiting for black line
	this.leftMotor.setSpeed(forwardSpeed);
	this.rightMotor.setSpeed(forwardSpeed);
	this.leftMotor.setAcceleration(500);
	this.rightMotor.setAcceleration(500);
	this.leftMotor.forward();
	this.rightMotor.forward();
	
	//Get sensor values
	lightValue.fetchSample(lsData, 0);
	float reading = lsData[0] ; 
	display.drawString("Color: " + reading, 0, 4);
	while (reading >= blackValue) {
	lightValue.fetchSample(lsData, 0);
	reading = lsData[0] ; //updating value
	display.drawString(" ", 0, 4);
	display.drawString("Color: " + reading, 0, 4);
	}
	
	
	//Stop when a black line is detected
	this.leftMotor.setSpeed(0);
	this.rightMotor.setSpeed(0);
	Sound.systemSound(false, 3);
	this.leftMotor.setSpeed(forwardSpeed);
	this.rightMotor.setSpeed(forwardSpeed);
	
	
	// turn clockwise 90deg and continue forward until next line is detected
	
	this.leftMotor.rotate(convertAngle(90, Lab4.TRACK, Lab4.WHEEL_RAD), true);
	this.rightMotor.rotate(-convertAngle(90, Lab4.TRACK, Lab4.WHEEL_RAD), false);
	this.leftMotor.forward();
	this.rightMotor.forward();
	lightValue.fetchSample(lsData, 0);
	reading = lsData[0]; 
	display.drawString("Color: " + reading, 0, 4);
	while (reading >= blackValue) {
	lightValue.fetchSample(lsData, 0);
	reading = lsData[0]; 
	display.drawString("Color: " + reading, 0, 4);
	}
	this.leftMotor.setSpeed(0);
	this.rightMotor.setSpeed(0);
	Sound.playNote(Sound.FLUTE, 880, 250);
	this.leftMotor.setSpeed(forwardSpeed);
	this.rightMotor.setSpeed(forwardSpeed);
	
	//rotation
	this.leftMotor.rotate(convertDistance(f, Lab4.WHEEL_RAD), true);
	this.rightMotor.rotate(convertDistance(f, Lab4.WHEEL_RAD), false);
	
	//counterclockwise rotation for 90 deg
	this.leftMotor.rotate(-convertAngle(90, Lab4.TRACK, Lab4.WHEEL_RAD), true);
	this.rightMotor.rotate(convertAngle(90, Lab4.TRACK, Lab4.WHEEL_RAD), false);
	
	// move "f" cm  
	this.leftMotor.rotate(convertDistance(f, Lab4.WHEEL_RAD), true);
	this.rightMotor.rotate(convertDistance(f, Lab4.WHEEL_RAD), false);
	
	// set all values to 0 when at origin
	this.odometer.setX(0);
	this.odometer.setY(0);
	this.odometer.setTheta(0);
	}
	
	private int convertDistance(double distance, double wheelRadius) {
		return (int) ((int) (180*distance) / (Math.PI*wheelRadius));
	}	

	private int convertAngle(double angle, double track, double wheelRadius){
		return convertDistance(Math.PI*track*angle/360, wheelRadius);
	}

}
