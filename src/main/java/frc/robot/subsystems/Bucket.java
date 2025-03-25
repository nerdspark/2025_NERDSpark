// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BucketConstants;

public class Bucket extends SubsystemBase {
  /** Creates a new Bucket. */
  private double[] prevDistancesLeft = new double[BucketConstants.timesForBucketToTestPositive];
  private double[] prevDistancesRight = new double[BucketConstants.timesForBucketToTestPositive];
  private int i = 0;

  public static boolean gripperHasGamePiece = false;
  private CANrange sensorLeft; 
  private CANrange sensorRight; 
  private CANrangeConfiguration sensorConfig;

  public Bucket() {
    sensorLeft = new CANrange(BucketConstants.leftSensorPort, ArmConstants.armCanBus);
    sensorRight = new CANrange(BucketConstants.rightSensorPort, ArmConstants.armCanBus);
    sensorConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs().withFOVRangeX(27).withFOVRangeY(27))
      .withProximityParams(new ProximityParamsConfigs().withProximityThreshold(Meters.of(BucketConstants.coralDistance)).withMinSignalStrengthForValidMeasurement(2500));

    sensorLeft.getConfigurator().apply(sensorConfig);
    sensorRight.getConfigurator().apply(sensorConfig);
  }

  @Override
  public void periodic() {
    refreshSensors();
    SignalLogger.writeBoolean("bucket detected", getDetected());
    SmartDashboard.putBoolean("bucket detected", getDetected());
    SmartDashboard.putNumber("bucket left sensor dist", sensorLeft.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("bucket right sensor dist", sensorRight.getDistance().getValueAsDouble());
    SmartDashboard.putNumberArray("prevdistancesleft",prevDistancesLeft);
    SmartDashboard.putBoolean("leftdetectedarray tripped", getLeftCoralDetected());
    SmartDashboard.putBoolean("left considered detected", sensorLeft.getIsDetected().getValue());
    // This method will be called once per scheduler run
  }
  public void refreshSensors() {
    i++;
    if (i >= BucketConstants.timesForBucketToTestPositive) {
      i = 0;
    }

    prevDistancesLeft[i] = sensorLeft.getIsDetected().getValue() ? sensorLeft.getDistance().getValueAsDouble() : 2.0;

    prevDistancesRight[i] = sensorRight.getIsDetected().getValue() ? sensorRight.getDistance().getValueAsDouble() : 2.0;

  }
  
  public boolean getLeftCoralDetected() {
    for (double distance : prevDistancesLeft) {
      if (distance > BucketConstants.coralDistance) {
        return false;
      }
    }
    return true;
  }
  public boolean getRightCoralDetected() {
    for (double distance : prevDistancesRight) {
      if (distance > BucketConstants.coralDistance) {
        return false;
      }
    }
    return true;
  }
  public boolean getDetected() {
    return (getLeftCoralDetected() || getRightCoralDetected());
  }
  public Command disableAutoBucket() {
    return new Command() {
      @Override
      public void initialize() {
        gripperHasGamePiece = true;
        }
    };
  }
}
