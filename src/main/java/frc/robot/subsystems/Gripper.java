// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;

public class Gripper extends SubsystemBase {
  private TalonFX gripper;
  private TalonFXConfiguration gripperConfig = new TalonFXConfiguration();
  private CANrange sensor; 
  private double[] prevDistances = new double[ArmConstants.timesToTestPositive];
  /** Creates a new Gripper. */
  public Gripper() {
    gripper = new TalonFX(ArmConstants.gripperMotorPort, ArmConstants.armCanBus);
    sensor = new CANrange(ArmConstants.gripperSensorPort, ArmConstants.armCanBus);
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.gripperCurrentLimitDefault)
          .withStatorCurrentLimitEnable(true);
        gripperConfig.Feedback = new FeedbackConfigs()
          .withFeedbackRotorOffset(ArmConstants.gripperOffset)
          .withSensorToMechanismRatio(ArmConstants.gripperRadPerRot);
        gripperConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);
        gripperConfig.Slot0 = new Slot0Configs()
          .withKP(ArmGains.gripperP)
          .withKI(ArmGains.gripperI)
          .withKD(ArmGains.gripperD);
        gripperConfig.MotorOutput = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        gripper
          .getConfigurator()
          .apply(gripperConfig);
  }
  
  public void setGripperPower(double power) {
    gripper.set(power); 
  }

  public void setCurrentLimit(double currentLimit) {
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(currentLimit)
          .withStatorCurrentLimitEnable(true);
    gripper.getConfigurator().apply(gripperConfig);
  }

  public double getCurrentLimit() {
    return gripperConfig.CurrentLimits.StatorCurrentLimit;
  }

  public boolean getCoralDetected() {
    for (double distance : prevDistances) {
      if (distance > ArmConstants.coralDistance) {
        return false;
      }
    }
    return true;
  }

  public boolean getAlgaeDetected() {
    for (double distance : prevDistances) {
      if (distance > ArmConstants.algaeDistance) {
        return false;
      }
    }
    return false;
  }

  @Override
  public void periodic() {
    for (int i = prevDistances.length-1; i > 0; i--) {
      prevDistances[i]  = prevDistances[i-1];
    }
    prevDistances[0] = sensor.getDistance().getValueAsDouble();

    
    SmartDashboard.putNumber("Gripper velocity", gripper.getVelocity().getValueAsDouble()); // angular velocity (rotations per second)

  }
}
