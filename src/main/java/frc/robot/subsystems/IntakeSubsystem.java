package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX motor = new TalonFX(33);

  public enum GamePiece {
    Cone,
    Cube
  }

  private GamePiece m_GamePiece;

  public IntakeSubsystem() {}

  @Override
  public void periodic() {}

  public void setGamePiece(GamePiece piece) {
    m_GamePiece = piece;
  }

  public void runIntake(double speed) {
    if (m_GamePiece == GamePiece.Cube) {
      if (speed > 0) {
        motor.set(TalonFXControlMode.PercentOutput, 0.2 * speed);
      } else {
        motor.set(TalonFXControlMode.PercentOutput, 0.5 * speed);
      }
    } else {
      motor.set(TalonFXControlMode.PercentOutput, -0.4 * speed);
    }
  }

  public void intakeGamePiece(GamePiece piece) {
    setGamePiece(piece);
    runIntake(-1);
  }

  public void outtakeGamePiece(GamePiece piece) {
    setGamePiece(piece);
    runIntake(1);
  }
}
