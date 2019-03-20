import org.letsbuildrocekts.libs.TimeOfFlightSensor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  private TimeOfFlightSensor tofsensor;

  @Override
  public void robotInit() {
    tofsensor = new TimeOfFlightSensor(0x620);
  }
  
  @Override
  public void teleopPeriodic() {
    if(tofsensor.inRange())
      System.out.println("distance: " + tofsensor.getDistance());
    else
      System.out.println("out of range");
  }
}
