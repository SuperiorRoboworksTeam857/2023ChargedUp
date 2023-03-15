package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(30, MotorType.kBrushless);

  private final AbsoluteEncoder m_absoluteEncoder;

  private static final double horizontalAngle = 0.68;
  private static final double substationAngle = 0.765;
  private static final double intakeAngle = 0.765;
  private static final double coneScoreAngle = 0.82;
  private static final double slightlyPastVerticalAngle = 0.9;
  private static final double verticalAngle = 0.950;

  public enum Positions {
    HORIZONTAL,
    SUBSTATION_INTAKE,
    INTAKE,
    TOP_CONE_SCORE,
    SLIGHTLY_OUT,
    VERTICAL
  }

  private static double deltaTime = 0.02;
  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(1, 40);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(5, 0.0, 0.0, m_constraints, deltaTime);

  private double m_goalAngle = verticalAngle;

  public WristSubsystem() {
    m_absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    double encoderPositionFactor = (2 * Math.PI); // radians
    double encoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    motor.getEncoder().setPositionConversionFactor(encoderPositionFactor);
    motor.getEncoder().setVelocityConversionFactor(encoderVelocityFactor);
  }

  @Override
  public void periodic() {
    m_controller.setGoal(m_goalAngle);
    motor.set(m_controller.calculate(m_absoluteEncoder.getPosition()));

    SmartDashboard.putNumber("wrist position", m_absoluteEncoder.getPosition());
    SmartDashboard.putNumber("wrist speed", m_absoluteEncoder.getVelocity());
  }

  public void goToAngle(Positions position) {
    switch (position) {
      case HORIZONTAL:
        m_goalAngle = horizontalAngle;
        break;
      case SUBSTATION_INTAKE:
        m_goalAngle = substationAngle;
        break;
      case INTAKE:
        m_goalAngle = intakeAngle;
        break;
      case TOP_CONE_SCORE:
        m_goalAngle = coneScoreAngle;
        break;
      case SLIGHTLY_OUT:
        m_goalAngle = slightlyPastVerticalAngle;
        break;
      case VERTICAL:
        m_goalAngle = verticalAngle;
        break;
    }
  }

  public boolean isWristAtGoal() {
    return Math.abs(m_absoluteEncoder.getPosition() - m_goalAngle) < 0.03;
  }

  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }
}
