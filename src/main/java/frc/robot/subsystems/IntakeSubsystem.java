package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
  TalonFX motor = new TalonFX(33);
  public IntakeSubsystem() {
  
  }

  @Override
  public void periodic() {
    
  }

  public void runIntake(double speed) {
    motor.set(TalonFXControlMode.PercentOutput, speed);
  }
}