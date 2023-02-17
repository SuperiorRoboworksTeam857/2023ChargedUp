package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;


public class StrafeToTargetCommand extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem m_limelight;
    private Timer timer = new Timer();
    private double timeout;
    private Joystick m_stick;
    public StrafeToTargetCommand(Swerve subsystem, LimelightSubsystem limelight, Joystick stick, double timeoutS){
        s_Swerve = subsystem;
        m_limelight = limelight;
        timeout = timeoutS;
        m_stick = stick;
        addRequirements(subsystem, limelight);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute(){
        boolean onTarget = false;

        double kP = 0.03;

        // -5 to point left of goal, +5 to point right of goal
        // +2 is pretty well centered
        double tx = m_limelight.getLimelightValue("tx");
    
        if (m_limelight.getLimelightValue("camMode") == 1) {
            m_limelight.toggleDriverCam();
        }
        double speed = MathUtil.clamp(tx * kP, -0.5, 0.5);
        if (Math.abs(tx) > 2.0 && timer.get() < timeout) {
            s_Swerve.drive(new Translation2d(0, speed), 0, false, true);
        } else {
            onTarget = true;
        }

        if (onTarget) {
            if (m_stick != null) {
                s_Swerve.drive(new Translation2d(-0.3 * m_stick.getY(), 0), 0, false, true);
            }
        }
    }

    @Override
    public void end(boolean inturrupted){
        s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}