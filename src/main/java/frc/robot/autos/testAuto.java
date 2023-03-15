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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class testAuto extends SequentialCommandGroup {
  public testAuto(RobotContainer robot) {

    PathPlannerTrajectory trajectory1 =
        PathPlanner.generatePath(
            new PathConstraints(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(0)),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(0)),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(0)));
    PathPlannerTrajectory trajectory2 =
        PathPlanner.generatePath(
            new PathConstraints(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(0)),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(150)),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(230), Units.inchesToMeters(0)),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(150)));
    // PathPlannerTrajectory trajectory3 =
    //     PathPlanner.generatePath(
    //         new PathConstraints(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared),
    //         new PathPoint(
    //             new Translation2d(Units.inchesToMeters(-230), Units.inchesToMeters(0)),
    // Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(150)),
    //         new PathPoint(
    //             new Translation2d(Units.inchesToMeters(-180), Units.inchesToMeters(0)),
    // Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(150)));
    PathPlannerTrajectory trajectory4 =
        PathPlanner.generatePath(
            new PathConstraints(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(230), Units.inchesToMeters(0)),
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(0)),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(6)),
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(0)));

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
    // PPSwerveControllerCommand swerveControllerCommand3 =
    //     new PPSwerveControllerCommand(
    //         trajectory3,
    //         robot.s_Swerve::getPose,
    //         Constants.Swerve.swerveKinematics,
    //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //         new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
    //         robot.s_Swerve::setModuleStates,
    //         robot.s_Swerve);
    PPSwerveControllerCommand swerveControllerCommand4 =
        new PPSwerveControllerCommand(
            trajectory4,
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
        // new InstantCommand(
        //     () -> robot.s_wrist.goToAngle(WristSubsystem.Positions.SLIGHTLY_OUT),
        //     robot.s_wrist),
        // new InstantCommand(() -> robot.s_elevator.goToPosition(ElevatorSubsystem.Positions.HIGH), robot.s_elevator),
        // new WaitUntilCommand(robot.s_elevator::isElevatorAtGoal),
        // new WaitCommand(1),
        // new InstantCommand(() -> robot.s_elevator.goToPosition(ElevatorSubsystem.Positions.FLOOR), robot.s_elevator),
        // new WaitUntilCommand(robot.s_elevator::isElevatorAtGoal),
        // swerveControllerCommand1,
        new TurnToAngleCommand(robot.s_Swerve, 150, 2),
        // swerveControllerCommand2,
        // new InstantCommand(() -> robot.s_Swerve.drive(new Translation2d(0, 0), 0, false, true)),
        // new WaitCommand(1),
        // new TurnToAngleCommand(robot.s_Swerve, 0, 2),
        // swerveControllerCommand4,
        new InstantCommand(() -> robot.s_Swerve.drive(new Translation2d(0, 0), 0, false, true)));
  }
}
