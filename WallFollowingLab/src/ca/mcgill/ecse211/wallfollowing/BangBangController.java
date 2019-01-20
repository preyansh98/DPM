package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private static final int FILTER_OUT = 20;
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

  @Override
  public void processUSData(int distance) {
	    int error = bandCenter - distance; //expected distance from wall (20) minus the current distance from the wall. 

	  if (distance >= 200 && filterControl  < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the
        // filter value
        filterControl++;
      } else if (distance >= 200) {
        // We have repeated large values, so there must actually be nothing
        // there: leave the distance alone
        this.distance = distance;
      } else {
        // distance went below 255: reset filter and leave
        // distance alone.
        filterControl = 0;
        this.distance = distance;
      }

    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
	  
	  //1. We need normal movement
	  //2. For sharp turn left
	  //3. For sharp turn right ****
	  //normal. 
  
	  
	    //when positive, too close
	    if(Math.abs(error) <= bandwidth){ //on the correct path
	    	//what does this code do?
	        WallFollowingLab.leftMotor.setSpeed(motorHigh); 
	        WallFollowingLab.rightMotor.setSpeed(motorHigh);
	        WallFollowingLab.leftMotor.forward();
	        WallFollowingLab.rightMotor.forward(); 
	    }
	    else if(error > 0){
	      //too close to the wall 	
	      //decrease rotation of outside wheel. right motor B 
	    	if(error > 20) {
	    		//way too close to the wall. back away
	    		WallFollowingLab.leftMotor.setSpeed(motorHigh);
	    		WallFollowingLab.rightMotor.stop();
	    		WallFollowingLab.leftMotor.forward();
	    	}
	    	else if(error > 15 && error < 20) {
	    		WallFollowingLab.leftMotor.setSpeed(motorHigh);
	    		WallFollowingLab.rightMotor.setSpeed(motorLow - 50); 
	    		WallFollowingLab.leftMotor.forward();
	    		WallFollowingLab.rightMotor.forward();
	    	}
	    	else {//normal distance from 10 to 20
	      WallFollowingLab.leftMotor.setSpeed(motorHigh); 
	      WallFollowingLab.rightMotor.setSpeed(motorLow); 
	      WallFollowingLab.leftMotor.forward();
	      WallFollowingLab.rightMotor.forward(); 
	    	}
	      }
	    else if(error < 0){ //too far from the wall
	    	//TODO: find a way to not make a huge turn. 
	    	//its turning into the wall
	    	if(error < -20 && error > -30) {
	    		WallFollowingLab.leftMotor.setSpeed(motorLow);
	    		WallFollowingLab.rightMotor.setSpeed(motorLow + 50);
	    		WallFollowingLab.leftMotor.forward();
	    		WallFollowingLab.rightMotor.forward();
	    	}
	    	else {
	      WallFollowingLab.leftMotor.setSpeed(motorLow); 
	      WallFollowingLab.rightMotor.setSpeed(motorHigh); 
	      WallFollowingLab.leftMotor.forward();
	      WallFollowingLab.rightMotor.forward(); 
	    	}
	    }
	    try{
	        Thread.sleep(50); 
	        }
	      catch(Exception e){
	      e.printStackTrace(); }
	    }


  @Override
  public int readUSDistance() {
    return this.distance;
  }
}