package ca.mcgill.ecse211.ultrasonic;


public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
