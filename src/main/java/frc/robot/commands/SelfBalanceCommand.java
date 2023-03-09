package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SelfBalanceCommand extends CommandBase {

  private final Swerve s_Swerve;

  public SelfBalanceCommand(Swerve subsystem) {
    s_Swerve = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // boolean onTarget = false;

    // double kP = 0.03;

    // // -5 to point left of goal, +5 to point right of goal
    // // +2 is pretty well centered
    // double tx = m_limelight.getLimelightValue("tx");

    // if (m_limelight.getLimelightValue("camMode") == 1) {
    //     m_limelight.toggleDriverCam();
    // }
    // double speed = MathUtil.clamp(tx * kP, -0.5, 0.5);
    // if (Math.abs(tx) > 2.0 && timer.get() < timeout) {
    //     s_Swerve.drive(new Translation2d(0, speed), 0, false, true);
    // } else {
    //     onTarget = true;
    // }
    // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
    // about 14...0...360...346
    // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and
    // back leveling
    // 0-14, drive forward, 346-360 drive backward

    // double angle = s_Swerve.getYFilteredAccelAngle();
    double angle = s_Swerve.getPitch();

    // SmartDashboard.putNumber("X rate", s_Swerve.gyro.getRawGyroX());
    // SmartDashboard.putNumber("Y rate", s_Swerve.gyro.getRawGyroY());
    // SmartDashboard.putNumber("Z rate", s_Swerve.gyro.getRawGyroZ());

    // if (angle > 180) {
    //     angle = -(360 - angle);
    // }
    if (Math.abs(s_Swerve.getPitchRate()) > 10) {
      s_Swerve.setX();
    } else if (angle > 4) {
      s_Swerve.drive(new Translation2d(-0.45, 0), 0, false, true);
    } else if (angle < -4) {
      s_Swerve.drive(new Translation2d(0.45, 0), 0, false, true);
    } else {
      s_Swerve.setX();
      // s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
    }
  }

  @Override
  public void end(boolean inturrupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
