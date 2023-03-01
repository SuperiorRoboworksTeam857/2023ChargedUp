/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {

  public LimelightSubsystem() {}

  @Override
  public void periodic() {}

  public boolean isTurnedToTarget() {
    boolean ans = false;

    if (Math.abs(getLimelightValue("tx")) < 4.0) {
      ans = true;
    }

    return ans;
  }

  public void turnOnDriverCam() {
    setLimelightValue("camMode", 1);
  }

  public void turnOffDriverCam() {
    setLimelightValue("camMode", 0);
  }

  public void toggleDriverCam() {
    double currentMode = getLimelightValue("camMode");

    if (currentMode == 0) {
      setLimelightValue("camMode", 1);
    } else if (currentMode == 1) {
      setLimelightValue("camMode", 0);
    }
  }

  public void enableLimelight(boolean enabled) {
    if (enabled) {
      setLimelightValue("ledMode", 0);
    } else {
      setLimelightValue("ledMode", 1);
    }
  }

  public double getLimelightValue(String entry) {

    double value = 0.0;

    if (entry.equals("tDistance")) {
      // value = (LimelightConstants.kTargetHeight - LimelightConstants.kLimelightHeight) /
      //   Math.tan(Math.toRadians(LimelightConstants.kLimelightAngle + getLimelightValue("ty")));
    } else {
      value =
          NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0);
    }

    return value;
  }

  public void setLimelightValue(String entry, double value) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setDouble(value);
  }

  public void setLimelightArray(String entry, double[] value) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setDoubleArray(value);
  }

  public enum Pipeline {
    RetroTape,
    AprilTags
  }

  public void setPipeline(Pipeline pipeline) {
    double value = 0.0;
    if (pipeline == Pipeline.RetroTape) {
      value = 0;
    } else if (pipeline == Pipeline.AprilTags) {
      value = 1;
    }
    setLimelightValue("pipeline", value);

    double[] cropValues = new double[4];
    cropValues[0] = -1.0;
    cropValues[1] = 1.0;
    cropValues[2] = 0.0;
    cropValues[3] = 1.0;
    setLimelightArray("crop", cropValues);
  }
}
