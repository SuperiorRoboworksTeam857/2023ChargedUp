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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.SelfBalanceCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ElevatorSubsystem.Positions;

public class AutoPreloadChargeStation extends SequentialCommandGroup {
  public AutoPreloadChargeStation(RobotContainer robot) {

    PathPlannerTrajectory trajectory1 =
        PathPlanner.generatePath(
            new PathConstraints(
                1,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
    PathPlannerTrajectory trajectory2 =
        PathPlanner.generatePath(
            new PathConstraints(
                1,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(0)), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(84), Units.inchesToMeters(0)), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)));
    
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
    PPSwerveControllerCommand swerveControllerCommand2 =
        new PPSwerveControllerCommand(
            trajectory2,
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
        
        new InstantCommand(() -> robot.s_elevator.goToPosition(Positions.HIGH), robot.s_elevator),
        new WaitUntilCommand(robot.s_elevator::isElevatorAtGoal),
        
        new WaitCommand(1),

        new InstantCommand(() -> robot.s_elevator.goToPosition(Positions.FLOOR), robot.s_elevator),
        new WaitUntilCommand(robot.s_elevator::isElevatorAtGoal),

        swerveControllerCommand1,

        new WaitCommand(0.5),

        swerveControllerCommand2,

        new ParallelRaceGroup(
            new SelfBalanceCommand(robot.s_Swerve),
            new WaitCommand(3)
        ),

        new InstantCommand(() -> robot.s_Swerve.drive(new Translation2d(0, 0), 0, false, true)));
  }
}
