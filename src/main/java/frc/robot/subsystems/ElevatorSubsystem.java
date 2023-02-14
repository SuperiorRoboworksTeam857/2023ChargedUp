package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

  private SlewRateLimiter slewLimiter = new SlewRateLimiter(3.0);


  CANSparkMax motor = new CANSparkMax(20, MotorType.kBrushless);
  public ElevatorSubsystem() {
  
  }

  @Override
  public void periodic() {
    
  }

  public void runElevator(double speed) {
    // -250 is top
    //    0 is bottom
    // negative speed goes up, positive speed goes down
    boolean movingUp = (speed < 0);
    boolean movingDown = (speed > 0);


    //speed = slewLimiter.calculate(speed);

//0, -110

    //motor.set(speed * 0.1);
    
    if ((motor.getEncoder().getPosition() < -15 && movingDown) ||
        (motor.getEncoder().getPosition() > -105 && movingUp)) {
      motor.set(speed*1);
    }
    else if ((motor.getEncoder().getPosition() < 0 && movingDown) ||
             (motor.getEncoder().getPosition() > -110 && movingUp)) {
      motor.set(speed*0.2);
    }
    // else if (movingDown || movingUp) {
    //   motor.set(speed*0.1);
    // }
    else {
      motor.set(0);
    }
    

    SmartDashboard.putNumber("elevator speed", speed);
    SmartDashboard.putNumber("elevator position", motor.getEncoder().getPosition());
  }


  
  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }
}