package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX motor = new TalonFX(33);
  private Spark ledPWM = new Spark(9);

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
    switch (m_GamePiece) {
      case Cone:
        setYellow();
        break;
      case Cube:
        setPurple();
        break;
    }
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

  private void setBlue() {
    ledPWM.set(0.87);
  }

  private void setPurple() {
    ledPWM.set(0.91);
  }

  private void setYellow() {
    ledPWM.set(0.69);
  }
}
