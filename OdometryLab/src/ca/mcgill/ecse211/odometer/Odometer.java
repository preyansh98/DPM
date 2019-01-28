/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  
  //The tacho counts refer to the rotations of the respective motors. 
  private int leftMotorTachoCount; //
  private int rightMotorTachoCount; //
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  
  double ThetaVal = 0; //A global variable for later to store the theta value calculated
  


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0; //initialized the current leftMotorTachoCount to 0
    this.rightMotorTachoCount = 0; //initialized the current rightMotorTachoCount to 0

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   * The robot knows in which direction it is going based on the number of tacho counts for the left motor and right motor.
   * It stores these tacho counts and on each iteration compares it to the old tacho count. 
   * 
   * The left distance, right distance, is calculated based on the difference of these tacho counts. 
   * The overall change in distance and change in the angle, theta, is calculated. This is then used to
   * calculate the specific changes in the X and Y direction using trigonometry. 
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();
      //initialize and store the current left and right tacho counts 
      int newLeftTacho = leftMotor.getTachoCount(); 
      int newRightTacho = rightMotor.getTachoCount(); 

      
        //new x, new y, and new theta compared to last tacho count. 
      // the following doubles create the distance covered by both wheels for every rotation   
      // the distances are calculated by multiplying the circumference (2*pi*r) by the difference in the tachocounts between the left and right motors, divided by 360 degrees. 
     double leftDist = Math.PI * WHEEL_RAD * (newLeftTacho - leftMotorTachoCount)/180; 
     double rightDist = Math.PI * WHEEL_RAD * (newRightTacho - rightMotorTachoCount)/180; 
     
     
     // updates old tachocounts to be new ones so the next iteration uses the new values
     this.leftMotorTachoCount = newLeftTacho;
     this.rightMotorTachoCount = newRightTacho;
     
     
     double deltaDist = 0.5*(leftDist + rightDist); //average of the two distances 
     double deltaTheta = (leftDist - rightDist)/(TRACK); //differences between distances covered by both wheels, divided by the distance between the wheels
     

     ThetaVal  += deltaTheta; 
     //sin and cos take care of the direction and sign of the X and Y values respectively.
     //For example when theta is 180 degrees, sin(180)=0, cos(180)=-1 so the Y value will decrease as the robot advances and the X value will not change.
     double changeInX = deltaDist * Math.sin(ThetaVal); 
     double changeInY = deltaDist * Math.cos(ThetaVal); 
     
      
      //updates the odometer with the new X and Y values as well as the new theta converted in degrees
     //the update method automatically adds the change to the original values
      odo.update(changeInX, changeInY, Math.toDegrees(deltaTheta)); //deltaTheta was converted to degrees since the calculations are in radians but display and correction in degrees 
       

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
