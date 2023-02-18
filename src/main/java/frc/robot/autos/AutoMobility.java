package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

public class AutoMobility extends SequentialCommandGroup {
  public AutoMobility(RobotContainer robot) {

    PathPlannerTrajectory trajectory1 =
        PathPlanner.generatePath(
            new PathConstraints(
                1,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
    
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(0, 2 * Math.PI);

    PPSwerveControllerCommand swerveControllerCommand1 =
        new PPSwerveControllerCommand(
            trajectory1,
            robot.s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
            robot.s_Swerve::setModuleStates,
            robot.s_Swerve);

    addCommands(
        new InstantCommand(
            () ->
                robot.s_Swerve.resetOdometry(
                    new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)))),
        swerveControllerCommand1,
        new InstantCommand(() -> robot.s_Swerve.drive(new Translation2d(0, 0), 0, false, true)));
  }
}
