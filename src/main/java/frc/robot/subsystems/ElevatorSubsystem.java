package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

  //private SlewRateLimiter slewLimiter = new SlewRateLimiter(3.0);

  private static double deltaTime = 0.02;
  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(20, 80);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(0.15, 0.0, 0.0, m_constraints, deltaTime);

  // or should we use SparkMaxPIDController?

  CANSparkMax motor = new CANSparkMax(20, MotorType.kBrushless);
  
  // Range of motion of 0 inches at bottom to -24.5 inches at top
  public enum Positions {
    FLOOR,
    MID,
    HIGH
  }

  private double m_goalPosition = 0;

  public ElevatorSubsystem() {
    double sprocketDiameter = 22 * 0.25 / Math.PI; // 22 teeth at 0.25 inch pitch
    double gearRatio = 25; // 25:1
    double driveConversionPositionFactor = (sprocketDiameter * Math.PI) / gearRatio;
    double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

    motor.getEncoder().setVelocityConversionFactor(driveConversionVelocityFactor);
    motor.getEncoder().setPositionConversionFactor(driveConversionPositionFactor);    
  }

  @Override
  public void periodic() {
    m_controller.setGoal(m_goalPosition);
    motor.set(m_controller.calculate(motor.getEncoder().getPosition()));

    SmartDashboard.putNumber("elevator position", motor.getEncoder().getPosition());
    SmartDashboard.putNumber("elevator speed", motor.getEncoder().getVelocity());
    
  }

  public void goToPosition(Positions position) {
    switch(position) {
      case FLOOR:
        m_goalPosition = 0;
        break;
      case MID:
        m_goalPosition = -12;
        break;
      case HIGH:
        m_goalPosition = -23.5;
        break;
    }
  }

  public boolean isElevatorAtGoal() {
    return Math.abs(motor.getEncoder().getPosition() - m_goalPosition) < 1.0;
  }
  
  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }
}