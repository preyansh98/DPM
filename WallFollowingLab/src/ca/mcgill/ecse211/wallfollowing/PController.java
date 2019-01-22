//package ca.mcgill.ecse211.wallfollowing;
//
//import lejos.hardware.motor.EV3LargeRegulatedMotor;
//
//public class PController implements UltrasonicController {
//
//  /* Constants */
//  private static final int MOTOR_SPEED = 200;
//  private static final int FILTER_OUT = 20;
//
//  private static final int THRESHOLD = 2; //2 cm 
//  private static final int WALL_DIST = 20; //20 cm from wall
//
//  private final int bandCenter;
//  private final int bandWidth;
//  private int distance; //initial distance 
//  private int filterControl; 
//
//  public PController(int bandCenter, int bandwidth) {
//    this.bandCenter = bandCenter;
//    this.bandWidth = bandwidth;
//    this.filterControl = 0;
//
//    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
//    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
//    WallFollowingLab.leftMotor.forward();
//    WallFollowingLab.rightMotor.forward();
//  }
//
//  @Override
//  public void processUSData(int distance) {
//
//    // rudimentary filter - toss out invalid samples corresponding to null
//    // signal.
//    // (n.b. this was not included in the Bang-bang controller, but easily
//    // could have).
//    //
//    if (distance >= 255 && filterControl < FILTER_OUT) {
//      // bad value, do not set the distance var, however do increment the
//      // filter value
//      filterControl++;
//    } else if (distance >= 255) {
//      // We have repeated large values, so there must actually be nothing
//      // there: leave the distance alone
//      this.distance = distance;
//    } else {
//      // distance went below 255: reset filter and leave
//      // distance alone.
//      filterControl = 0;
//      this.distance = distance;
//    }
//
//    // TODO: process a movement based on the us distance passed in (P style)
//  }
//
//
//  @Override
//  public int readUSDistance() {
//    return this.distance;
//  }
//
//}

package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 16;

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
		if (distance >= 90 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 90) {
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

		// calculating the speed in proportion to the error
		int error = Math.abs(this.distance - bandCenter);
		int deltaspeed = errorConstant * error; // adjusting speed proportional to the error using a constant,
												// errorConstant.
		if (deltaspeed > 200) { // max speed as 200
			deltaspeed = 200;
		}
		// case 1: Too far from wall, machine will adjust direction inward to avoid
		// touching the wall (Left turn)
		if (this.distance > (bandCenter + bandWidth)) {
			if(this.distance > 2.5*(bandCenter + bandWidth)) {
				WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED - (1.8)*deltaspeed));
				WallFollowingLab.rightMotor.setSpeed((int) (1.3*MOTOR_SPEED));
				WallFollowingLab.leftMotor.forward();
				WallFollowingLab.rightMotor.forward();
			}
			else {
			WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED - (1.8)*deltaspeed)); //speed - +ve speed  //100
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); //right is going faster
			WallFollowingLab.rightMotor.forward();
			WallFollowingLab.leftMotor.forward();}
		}
		// case 2: Kinda close to wall,
		else if (this.distance < (bandCenter + bandWidth) && this.distance >= 10) {
			WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED + (.8)*deltaspeed)); //speed - -ve deltaspeed so left is increasing //180
			WallFollowingLab.rightMotor.setSpeed((int) (MOTOR_SPEED - (1.3)*deltaspeed)); //speed is reducing. speed + -ve delta //100
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		// case 3: Way too close to wall,
		else if (this.distance < 10) {
			WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED - (4*deltaspeed))); //speed - -ve delta //270
			//WallFollowingLab.rightMotor.setSpeed(120); //speed + -ve delta
			WallFollowingLab.rightMotor.stop();
			WallFollowingLab.leftMotor.forward();
		}

		// Correct distance from wall, goes straight.
		else {
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
