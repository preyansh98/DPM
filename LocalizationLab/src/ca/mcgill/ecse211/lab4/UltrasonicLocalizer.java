package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.ultrasonic.UltrasonicController;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;



 /**
  * 
  * @author Preyansh & Maxime
  * 
  *this class allows the robot to detect the left and back walls using the Ultrasonic sensor and therefore localize itself.
  *There are two ways to detect the walls. Either by using the falling edge method or by using the rising edge method. Both are explained below
  */
 
public class UltrasonicLocalizer {
    
    //Setting up motors
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    
    //Ultrasonic sensor initializers
    private float[] usData;
    private SampleProvider usSamples;
       
    //Other global variables
    private final double TRACK = 0;
    private final double WHEEL_RAD = 0;
    private static final int ROTATE_SPEED = 50; //rotate speed significantly lowered to support sample frequency
    private double deltaT = 0; 
    
    private static final int WALL_DISTANCE = 18; //distance sensor detects before turning away
    private int errorMargin = 3;//error margin is introduced to counter the effect of noise
    private double TURNING_ERROR = 3.5;
    
    
    //Odometer initializers
    private Odometer odometer;
    
    //Enum created to distinguish between whether to start with rising edge or falling edge
    public enum edgeType {
        FallingEdge, RisingEdge;
    }
    
    private edgeType type;//edgeType initializer
    
    /**
     * This is the constructor of the class. It sets up both motors, the odometer, the track between the wheels, the radius of the wheels, the edgetype and the ultrasonic sensor
     * @param leftMotor
     * @param rightMotor
     * @param odometer
     * @param TRACK
     * @param WHEEL_RAD
     * @param type
     * @param usSamples
     * @throws OdometerExceptions
     */
    public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
                               Odometer odometer, final double TRACK, final double WHEEL_RAD, edgeType type,
                               SampleProvider usSamples) throws OdometerExceptions {
        
        //Inheriting references to the motors first
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        
        //Setting up odometer 
        this.odometer = odometer;
        
        //Setting up whether falling edge or rising edge
        this.type = type;
        
        //Setting up ultrasonic sensor
        this.usSamples = usSamples;
        this.usData = new float[usSamples.sampleSize()];
        
        
    }
    /**
     * This is the main method. It calls the fallingEdge() and risingEdge() methods when needed
     */
    public void mainMethod() {
        //The main threaded method.
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        
        
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
    
    /**
     * This method detects a wall when the distance is starting to decrease, then turns away until distance falls again (i.e. other wall detected). 
     * The robot now knows where the two walls are (same distance for both detection) and can therefore orient itself to the 0 degree axis (i.e. straight forward)
     */
    public void fallingEdge() {
        
        //1. Rotate to face away from the wall
        while(getDistance() < WALL_DISTANCE + errorMargin) {
            turnCounterclockwise();//call method to turn counter-clockwise
            //rotate counter clockwise
        }
        
        //2. Rotate to find the wall
        while(getDistance() > WALL_DISTANCE) {
            turnCounterclockwise();//call method to turn counter-clockwise
            
        }
        Sound.beep();
        double firstAngle = odometer.getXYT()[2];//store first theta value in a double variable
        
        //3. Rotate outside of the wall now
        while(getDistance() < WALL_DISTANCE + errorMargin) {
            turnClockwise();//call method to turn clockwise
        }
        
        //4. Find the second wall now
        while(getDistance() > WALL_DISTANCE) {
            turnClockwise();//call method to turn clockwise
        }
        Sound.beep();
        double secondAngle = odometer.getXYT()[2];//store second theta value in a double variable
        
        stop();//call stop method to stop the robot
        
        //create a deltaT variable to later add to the actual theta value to be able to turn to the 0 degree axis properly
        //implement 2 cases  depending on which angle is bigger than the other
        if(firstAngle > secondAngle) {
            deltaT = 225 - (firstAngle + secondAngle)/2;
        }
        else {
            deltaT = 45 - (firstAngle + secondAngle)/2;
        }
        
        
        double turnAngle = deltaT + odometer.getXYT()[2];//add deltaT and theta value as described above
        
        //rotate to 0 degree axis
        leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turnAngle-TURNING_ERROR), true);
        rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turnAngle-TURNING_ERROR), false);
        
        // set odometer to theta = 0
        odometer.setXYT(0.0, 0.0, 0.0);
        
    }
    /**
     * This method makes the robot turn to a specific angle entered.
     * @param theta
     */
    public void turnTo(double theta) {
        
        int rotation = convertAngle(theta, TRACK, WHEEL_RAD);
        leftMotor.rotate(rotation, true);
        rightMotor.rotate(-rotation, false);
        stop();//call method to stop robot
    }
    
    /**
     * This method detects a wall when the distance is starting to increase, then turns away until distance rises again (i.e. other wall detected). 
     * The robot now knows where the two walls are (same distance for both detection) and can therefore orient itself to the 0 degree axis (i.e. straight forward)
     */
    public void risingEdge() {
        
        //1. Rotate to the wall
        while(getDistance() > WALL_DISTANCE) {
            turnCounterclockwise();//call method to turn counter-clockwise
        }
        
        //2. Rotate until open space
        while(getDistance() < WALL_DISTANCE + errorMargin) {
            turnCounterclockwise();//call method to turn counter-clockwise
        }
        Sound.beep();
        double firstAngle = odometer.getXYT()[2];//store first theta value in a double variable
        
        //3. Rotate until sees the wall
        while(getDistance() > WALL_DISTANCE) {
            turnClockwise();//call method to turn clockwise
        }
        
        //4. Rotate until open space again
        while(getDistance() < WALL_DISTANCE + errorMargin) {
            turnClockwise();//call method to turn clockwise
        }
        Sound.beep();
        double secondAngle = odometer.getXYT()[2];//store second theta value in a double variable
        
        stop();//stop robot
        
      //create a deltaT variable to later add to the actual theta value to be able to turn to the 0 degree axis properly
        //implement 2 cases  depending on which angle is bigger than the other
        if(firstAngle > secondAngle) {
            deltaT = 225 - (firstAngle + secondAngle)/2 + 180;
        }
        else {
            deltaT = 45 - (firstAngle + secondAngle)/2 + 180;
        }
        
        double turnAngle = deltaT + odometer.getXYT()[2];//add deltaT and theta value as described above
        
        //rotate to 0 degree axis
        leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turnAngle-TURNING_ERROR), true);
        rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turnAngle-TURNING_ERROR), false);
        
        // set theta = 0.0
        odometer.setXYT(0.0, 0.0, 0.0);
    }
           
      /**
       * this method gets the distance samples from the ultrasonic sensor
       * @return
       */
    public int getDistance() {
        usSamples.fetchSample(usData, 0);
        int distance = (int) (usData[0] * 100);
        return distance;
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
    
    /**
     * This method takes in the radius, width, and angle and converts the angle to a 
     * tacho count to rotate the wheels. 
     * @param radius
     * @param width
     * @param angle
     * @return an integer tacho count of how much the motors should rotate to get this angle
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    /**
     * this method allows the robot to turn clockwise
     */
    public void turnClockwise() {
        leftMotor.forward();
        rightMotor.backward();
    }
    
    /**
     * this method allows the robot to turn counterclockwise
     */
    public void turnCounterclockwise() {
        leftMotor.backward();
        rightMotor.forward();
    }
    
    /**
     * this method allows the robot to stop
     */
    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }
    
}