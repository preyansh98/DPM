package ca.mcgill.ecse211.wallfollowing;

/**

This class is for the PController (i.e. proportional controller) functionality. 
The PController sets the relative change in the speeds of left and right motors
according to the distance error monitored by the Ultrasonic Sensor. 

@modified by Preyansh, Maxime

**/

public class BangBangController implements UltrasonicController {

  private static final int FILTER_OUT = 17; //The filter_out refers to number of continous samples the sensor will effectively "filter out" as not the real distance measured. This takes into account gaps
  private int filterControl; 
 private final int bandCenter; //offset from the wall
  private final int bandwidth; //width of the deadband i.e. error margin
  private final int motorLow; //speed of lower -- deltaspeed
  private final int motorHigh; //speed of high -- fwdspeed
  private int distance; 
  
  
  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    filterControl = 0; 
  }

	/** 
	 * 
	 * This method provides a BangBang-style implementation of how the Ultrasonic Sensor
	 * should process distance data, and move the left and right motors in respect to keeping
	 * a fixed distance bandCenter from the wall. 
	 * 
	 * @param: distance that Ultrasonic Sensor detects.
	 */
  @Override
  public void processUSData(int distance) {
	    int error = bandCenter - distance; //expected distance from wall (30) minus the current distance from the wall. 

	  if (distance >= 150 && filterControl  < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the
        // filter value
        filterControl++;
      } else if (distance >= 150) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone and pass it to the processUSData
    	  //since the sensor is at 45 degree angle, we divide the distance by approximately cos(45) factor to get the horizontal distance instead
        this.distance = (int) ((distance)/Math.sqrt(2));
      } else {
        // distance went below 150: reset filter and leave
        // distance alone.
        filterControl = 0;
        this.distance = (int) ((distance)/Math.sqrt(2));
      }

    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
	  
	    //When the error is less than the bandwidth, it is on the correct path
	    if(Math.abs(error) <= bandwidth){ 
	        WallFollowingLab.leftMotor.setSpeed(motorHigh); 
	        WallFollowingLab.rightMotor.setSpeed(motorHigh);
	        WallFollowingLab.leftMotor.forward();
	        WallFollowingLab.rightMotor.forward(); 
	    }
	    else if(error > 0){
	      //Case 1: The EV3 is close to the wall. It must make a right turn. The correction varies depending on the size of the error (i.e. how close to the wall)
	    	if(error > 18) {
	    		//When the Error is really huge, the EV3 is too close to the wall
	    		//The right motor stops, and left motor on high to spin away from touching the wall
	    		WallFollowingLab.leftMotor.setSpeed(2*motorHigh + 50);
	    		WallFollowingLab.rightMotor.stop();
	    		WallFollowingLab.leftMotor.forward();
	    	}
	    	else if(error > 10 && error < 18) {
	    		//If there is an error, but not too close to the wall, the left motor moves faster relative to right Motor 
	    		//to make sure the EV3 does not crash into the wall. Correction is not as strict as first case when error too huge
	    		WallFollowingLab.leftMotor.setSpeed(motorHigh + motorLow + motorLow);
	    		WallFollowingLab.rightMotor.setSpeed(motorLow); 
	    		WallFollowingLab.leftMotor.forward();
	    		WallFollowingLab.rightMotor.forward();
	    	}
	    	else {
	    		//very small error in being close to the wall, leftMotor moves away slower to not apply a huge correction
	      WallFollowingLab.leftMotor.setSpeed(motorHigh + 35); 
	      WallFollowingLab.rightMotor.setSpeed(motorLow); 
	      WallFollowingLab.leftMotor.forward();
	      WallFollowingLab.rightMotor.forward(); 
	    	}
	      }
	    else if(error < 0){ //The EV3 is far from the wall, so must make a left turn.
	    	//Again, correction varies depending on the size of the error (i.e. how far from the wall)
	    	if(error < -5 && error > -20) {
	    		//The EV3 is not too far from the wall but should turn in to the left
	    		//so that the EV3 gets close and applies a correction to move away
	    		WallFollowingLab.leftMotor.setSpeed(motorHigh);
	    		WallFollowingLab.rightMotor.setSpeed(motorHigh + 50);
	    		WallFollowingLab.leftMotor.forward();
	    		WallFollowingLab.rightMotor.forward();
	    	}
	    	else if(error > -20 && error < -40) {
	    		//The EV3 is in the middle range far from the wall so should move straight
	    		//until a correction needs to be made
	    		WallFollowingLab.leftMotor.setSpeed(motorHigh);
	    		WallFollowingLab.rightMotor.setSpeed(motorHigh);
	    		WallFollowingLab.leftMotor.forward();
	    		WallFollowingLab.rightMotor.forward();
	    	}
	    	else {
	    		//The EV3 is detecting a huge distance so should move in to the right slowly
	    		//This was done because on the outside concave turns, distance measured
	    		//alternates between high and low so moves in slow
	      WallFollowingLab.leftMotor.setSpeed(motorLow); 
	      WallFollowingLab.rightMotor.setSpeed(motorLow + 50); 
	      WallFollowingLab.leftMotor.forward();
	      WallFollowingLab.rightMotor.forward(); 
	    	}
	    }
	    try{
	        Thread.sleep(50); 
	        //After program is run, the thread is told to sleep so other processes can access CPU
	        }
	      catch(Exception e){
	      e.printStackTrace(); }
	    }


  @Override
  public int readUSDistance() {
    return this.distance;
  }
}