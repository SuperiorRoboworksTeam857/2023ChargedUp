package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private Spark ledPWM = new Spark(9);

  public LEDSubsystem() {}

  @Override
  public void periodic() {}

  public void setBlue() {
    ledPWM.set(0.87);
  }

  public void setPurple() {
    ledPWM.set(0.91);
  }

  public void setYellow() {
    ledPWM.set(0.69);
  }
}
