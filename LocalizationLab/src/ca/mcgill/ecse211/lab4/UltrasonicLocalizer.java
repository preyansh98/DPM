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


/*TODO:
 * 1. Make this class extend the ultrasonic poller and use that instead
 * 2. Make methods for clockwise rotation and anti-clockwise
 * 3. Make method for stopping.
 */
public class UltrasonicLocalizer {
    
    //Setting up motors
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    
    //Ultrasonic sensor initializers
    private float[] usData;
    private SampleProvider usSamples;
    
    //Ultrasonic sensor global variables to process distance
    private int distance;
    private static final int FILTER_OUT = 35;
    private int filterControl = 0;
    
    //Other global variables
    private final double TRACK = 0;
    private final double WHEEL_RAD = 0;
    private static final int MOTOR_SPEED = 100;
    private static final int ROTATE_SPEED = 50; //rotate speed significantly lowered to support sample frequency
    private double deltaT = 0;
    
    private int fallingMin = 18;
    private int risingMax = 45;
    private int errorMargin = 3;
    private double TURNING_ERROR = 3.5;
    
    //Ultrasonic Sensor
    
    //Odometer initializers
    private Odometer odometer;
    private OdometerData odoData;
    
    //Enum created to distinguish between whether to start with rising edge or falling edge
    public enum edgeType {
        FallingEdge, RisingEdge;
    }
    
    private edgeType type;
    
    public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
                               Odometer odometer, final double TRACK, final double WHEEL_RAD, edgeType type,
                               SampleProvider usSamples) throws OdometerExceptions {
        
        //Inheriting references to the motors first
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        
        //Setting up odometer to 0,0,0
        this.odometer = odometer;
        
        //Setting up whether falling edge or rising edge
        this.type = type;
        
        //Setting up ultrasonic sensor
        this.usSamples = usSamples;
        this.usData = new float[usSamples.sampleSize()];
        
        
    }
    
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
    
    public void fallingEdge() {
        
        //1. Rotate to face away from the wall
        while(getDistance() < fallingMin + errorMargin) {
            turnCounterclockwise();//call method to turn counter-clockwise
            //rotate counter clockwise
        }
        
        //2. Rotate to find the wall
        while(getDistance() > fallingMin) {
            turnCounterclockwise();//call method to turn counter-clockwise
            
        }
        Sound.beep();
        double firstAngle = odometer.getXYT()[2];
        
        //3. Rotate outside of the wall now
        while(getDistance() < fallingMin + errorMargin) {
            turnClockwise();//call method to turn clockwise
        }
        
        //4. Find the second wall now
        while(getDistance() > fallingMin) {
            turnClockwise();//call method to turn clockwise
        }
        Sound.beep();
        double secondAngle = odometer.getXYT()[2];
        
        stop();//call stop method to stop the robot
        
        if(firstAngle > secondAngle) {
            deltaT = 225 - (firstAngle + secondAngle)/2;
        }
        else {
            deltaT = 45 - (firstAngle + secondAngle)/2;
        }
        
        double turnAngle = deltaT + odometer.getXYT()[2];
        
        leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turnAngle-TURNING_ERROR), true);
        rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turnAngle-TURNING_ERROR), false);
        
        // set odometer to theta = 0
        odometer.setXYT(0.0, 0.0, 0.0);
        
    }
    
    public void turnTo(double theta) {
        
        int rotation = convertAngle(theta, TRACK, WHEEL_RAD);
        leftMotor.rotate(rotation, true);
        rightMotor.rotate(-rotation, false);
        stop();//call method to stop robot
    }
    
    public void risingEdge() {
        
        //1. Rotate to the wall
        while(getDistance() > fallingMin) {
            turnCounterclockwise();//call method to turn counter-clockwise
        }
        
        //2. Rotate until open space
        while(getDistance() < fallingMin + errorMargin) {
            turnCounterclockwise();//call method to turn counter-clockwise
        }
        Sound.beep();
        double firstAngle = odometer.getXYT()[2];
        
        //3. Rotate until sees the wall
        while(getDistance() > fallingMin) {
            turnClockwise();//call method to turn clockwise
        }
        
        //4. Rotate until open space again
        while(getDistance() < fallingMin + errorMargin) {
            turnClockwise();//call method to turn clockwise
        }
        Sound.beep();
        double secondAngle = odometer.getXYT()[2];
        
        stop();//stop robot
        
        if(firstAngle > secondAngle) {
            deltaT = 225 - (firstAngle + secondAngle)/2 + 180;
        }
        else {
            deltaT = 45 - (firstAngle + secondAngle)/2 + 180;
        }
        
        double turnAngle = deltaT + odometer.getXYT()[2];
        
        leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turnAngle-TURNING_ERROR), true);
        rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turnAngle-TURNING_ERROR), false);
        
        // set theta = 0.0
        odometer.setXYT(0.0, 0.0, 0.0);
    }
    
    
    
    public void processUSData(int distance) {
        
        // rudimentary filter - toss out invalid samples corresponding to null
        // (n.b. this was not included in the Bang-bang controller, but easily
        // could have).
        //
        if (distance >= 255 && filterControl < FILTER_OUT) {
            // bad value, do not set the distance var, however do increment the
            // filter value to read continous samples until Filter_out is met
            filterControl++;
        } else if (distance >= 255) {
            // We have repeated large values, so there must actually be nothing
            // there: leave the distance alone and pass it to the processUSData
            this.distance = distance;
        } else {
            // distance went below 90: reset filter and leave
            // distance alone.
            filterControl = 0;
            this.distance = distance;
        }
        LocalEV3.get().getTextLCD().drawString("distance: " + distance, 0, 6);
        
        
    }
    
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
    
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    public void turnClockwise() {
        leftMotor.forward();
        rightMotor.backward();
    }
    
    public void turnCounterclockwise() {
        leftMotor.backward();
        rightMotor.forward();
    }
    
    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }
    
}
