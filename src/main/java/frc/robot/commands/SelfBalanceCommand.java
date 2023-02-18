package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;


public class SelfBalanceCommand extends CommandBase {

    private final Swerve s_Swerve;

    public SelfBalanceCommand(Swerve subsystem){
        s_Swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }
    
    @Override
    public void execute(){
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
  //SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between about 14...0...360...346
  //SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and back leveling
  // 0-14, drive forward, 346-360 drive backward

        //double angle = s_Swerve.getYFilteredAccelAngle();
        double angle = s_Swerve.getPitch();

        // if (angle > 180) {
        //     angle = -(360 - angle);
        // }
        
        if (angle > 4) {
            s_Swerve.drive(new Translation2d(-0.3, 0), 0, false, true);
        } else if (angle < -4) {
            s_Swerve.drive(new Translation2d(0.3, 0), 0, false, true);
        } else {
            s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
        }
    }

    @Override
    public void end(boolean inturrupted){
        s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}