package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnToAngleCommand extends CommandBase {

  private final Swerve s_Swerve;
  private boolean complete = false;
  private double angle;
  private Timer timer = new Timer();
  private double timeout;

  public TurnToAngleCommand(Swerve subsystem, double degrees, double timeoutS) {
    s_Swerve = subsystem;
    angle = degrees;
    timeout = timeoutS;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    complete = false;
  }

  @Override
  public void execute() {
    double gyroAngle = s_Swerve.getYaw().getDegrees();  // do we need to negate this number?

    final double kP = 0.2;
    SmartDashboard.putNumber("TurnToAngle - gyroAngle", gyroAngle);
    SmartDashboard.putNumber("TurnToAngle - goal angle", angle);

    if (angle > 180) {
      angle = -(360 - angle);
    } else if (angle < -180) {
      angle = 360 + angle;
    }

    SmartDashboard.putNumber("TurnToAngle - corrected goal angle", angle);

    double err = angle - gyroAngle;
    double speed =
        MathUtil.clamp(
            err * kP,
            -Constants.Swerve.maxAngularVelocity * 0.5,
            Constants.Swerve.maxAngularVelocity * 0.5);

    SmartDashboard.putNumber("TurnToAngle - speed", speed);


    if ((Math.abs(err) > 2 || Math.abs(s_Swerve.getYawRate()) > 1) && timer.get() < timeout) {
      s_Swerve.drive(new Translation2d(0, 0), speed, false, true);
    } else {
      complete = true;
    }
  }

  @Override
  public void end(boolean inturrupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return complete;
  }
}
