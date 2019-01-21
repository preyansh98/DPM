package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  
  private int errorConstant = 5;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    
 // Calculation of speed adjustment using error
    int error = Math.abs(this.distance - bandCenter);
	int speed_adjustment = errorConstant * error; //adjusting speed proportional to the error, using a constant.
	if (speed_adjustment > 200) { //setting max speed
		speed_adjustment = 200;
	}
	// case 1: Too far from wall, machine will adjust direction inward to avoid touching the wall (Left turn)
	if (this.distance > (bandCenter + bandWidth)) {
		WallFollowingLab.rightMotor.forward();
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + speed_adjustment/5);
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-(speed_adjustment/3));
	}
	// case 2: Too close to wall, machine will adjust direction outward to avoid touching the wall (Right turn)
	else if (this.distance < (bandCenter + bandWidth) && this.distance >= 26) {
		WallFollowingLab.rightMotor.forward();
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - (speed_adjustment / 2));
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + speed_adjustment/3);
	}
	// case 3: Way too close to wall, if the distance is less than 21 cm the machine will reverse its outer wheel to avoid touching the wall
	else if (this.distance < 22) {
			WallFollowingLab.rightMotor.backward();
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - speed_adjustment);
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + (4/3)*speed_adjustment);
	}
	
	//Correct distance from wall, goes straight. 
	 else {
		WallFollowingLab.rightMotor.forward();
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	}
}

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
