package ca.mcgill.ecse211.wallfollowing;

/**

	This class is for the PController (i.e. proportional controller) functionality. 
	The PController sets the relative change in the speeds of left and right motors
according to the distance error monitored by the Ultrasonic Sensor. 

@modified by Preyansh, Maxime

**/

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200; //a final speed of motor set as 200 rpm
	private static final int FILTER_OUT = 16; //The filter_out refers to number of continous samples the sensor will effectively "filter out" as not the real distance measured. This takes into account gaps

	private final int bandCenter; //fixed distance from the wall required
	private final int bandWidth; //margin of error that is allowed
	private int distance; //actual distance
	private int filterControl; //a simple counter for samples

	private int errorConstant = 5; //Since PController is proportional a numeric factor is chosen as a constant k 

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;
		//The PController constructor takes in arguments from the main code, and initializes this class with set value of bandCenter and bandWidth
		
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
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

		// rudimentary filter - toss out invalid samples corresponding to null
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 90 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value to read continous samples until Filter_out is met
			filterControl++;
		} else if (distance >= 90) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone and pass it to the processUSData
			this.distance = distance;
		} else {
			// distance went below 90: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// TODO: process a movement based on the us distance passed in (P style)

		// calculating the speed in proportion to the error
		int error = Math.abs(this.distance - bandCenter);
		
		int deltaspeed = errorConstant * error; // adjusting speed proportional to the error using a constant: errorConstant.
		if (deltaspeed > MOTOR_SPEED) //this if statement is added to ensure that maximum change in speed is upper bounded by motor_speed. 
		{
			deltaspeed = MOTOR_SPEED;
		}
		//Case 1: Too far away from wall, EV3 should move inward to avoid touching the wall
		//i.e. a left turn.
		if (this.distance > (bandCenter + bandWidth)) {
			
			if(this.distance > 2.5*(bandCenter + bandWidth)) 
			/*This code runs if the EV3 is relatively really far away from the wall.
			In this case, the rightMotor should move relatively faster than the leftMotor
			to bring the EV3 closer to the path */
			{
				WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED - (1.8)*deltaspeed)); 
				WallFollowingLab.rightMotor.setSpeed((int) (1.3*MOTOR_SPEED)); 
				WallFollowingLab.leftMotor.forward();
				WallFollowingLab.rightMotor.forward();
			}
			else {
			/*In the case the EV3 is far, but not TOO far, rightMotor moves faster than LeftMotor
			 * but at a small relative difference between the two so that the EV3 does not crash into the wall.
			 */
			WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED - (1.8)*deltaspeed)); 
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); //right is going faster
			WallFollowingLab.rightMotor.forward();
			WallFollowingLab.leftMotor.forward();}
		}
		//Case 2: Close to the wall, but not too much so moves right slowly
		else if (this.distance < (bandCenter + bandWidth) && this.distance >= 10) {
			WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED + (.8)*deltaspeed)); 
			WallFollowingLab.rightMotor.setSpeed((int) (MOTOR_SPEED - (1.3)*deltaspeed)); 
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
		//Case 3: Way too close to wall, one motor stops and the other makes it spin away
		//to avoid touching the wall
		else if (this.distance < 10) {
			WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED - (4*deltaspeed))); 
			//WallFollowingLab.rightMotor.setSpeed(120); 
			WallFollowingLab.rightMotor.stop();
			WallFollowingLab.leftMotor.forward();
		}

		// Correct distance from wall within bandwidth, goes straight.
		else {
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
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
