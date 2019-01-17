package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter; //offset from the wall
  private final int bandwidth; //width of the deadband
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
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)

    int error = bandCenter - distance; //expected distance from wall (20) minus the current distance from the wall. 
    //when positive, too close
    if(Math.abs(error) <= bandwidth){
        WallFollowingLab.leftMotor.setSpeed(motorHigh); 
        WallFollowingLab.rightMotor.setSpeed(motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.leftMotor.forward(); 
    }
    else if(error > 0){
      //too close to the wall 
      //decrease rotation of outside wheel. right motor B 
      WallFollowingLab.leftMotor.setSpeed(motorHigh); 
      WallFollowingLab.rightMotor.setSpeed(motorHigh - motorLow); 
      leftMotor.forward(); 
      rightMotor.forward(); 

    }
    else{ //too far from the wall
      WallFollowingLab.leftMotor.setSpeed(motorHigh); 
      WallFollowingLab.rightMotor.setSpeed(motorHigh + motorLow); 
      leftMotor.forward(); 
      rightMotor.forward(); 
    }
    Thread.sleep(50); 
    }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}