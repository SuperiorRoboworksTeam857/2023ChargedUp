package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase{

  //private SlewRateLimiter slewLimiter = new SlewRateLimiter(3.0);


  CANSparkMax motor = new CANSparkMax(30, MotorType.kBrushless);
  public WristSubsystem() {
  
  }

  @Override
  public void periodic() {
    
  }

  public void runWrist(double speed) {
    // -250 is top
    //    0 is bottom
    // negative speed goes up, positive speed goes down
    boolean movingUp = (speed < 0);
    boolean movingDown = (speed > 0);


    //speed = slewLimiter.calculate(speed);

//0, -110

    //motor.set(speed * 0.1);
    
    // if ((motor.getEncoder().getPosition() < 90 && movingDown) ||
    //     (motor.getEncoder().getPosition() > -90 && movingUp)) {
      motor.set(speed*.2);
    // }
    // else {
    //   motor.set(0);
    // }
    

    SmartDashboard.putNumber("wrist speed", speed);
    SmartDashboard.putNumber("wrist position", motor.getEncoder().getPosition());
  }


  
  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }
}