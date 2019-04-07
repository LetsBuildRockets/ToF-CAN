package frc.robot;

import org.letsbuildrockets.libs.TimeOfFlightSensor;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private TimeOfFlightSensor tofsensor;

  @Override
  public void robotInit() {
    tofsensor = new TimeOfFlightSensor(0x621);
    System.out.println("ToF Sensor at " + String.format("0x%03x", tofsensor.getID()) + "! (with firmware version: " + tofsensor.getFirwareVersion().toString()+")");
  }
  
  @Override
  public void teleopPeriodic() {
    if(tofsensor.inRange()) {
      System.out.println("distance: " + tofsensor.getDistance()+ " " + tofsensor.getError());
    } else {
      System.out.println("out of range");
    }
  }

}
