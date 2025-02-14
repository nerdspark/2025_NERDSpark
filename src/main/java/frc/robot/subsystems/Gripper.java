// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;

public class Gripper extends SubsystemBase {
  private TalonFX gripper;
  private TalonFXConfiguration gripperConfig = new TalonFXConfiguration();

  /** Creates a new Gripper. */
  public Gripper() {
    gripper = new TalonFX(ArmConstants.gripperMotorPort, "rio");
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperOpen)
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
        .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        gripper
          .getConfigurator()
          .apply(gripperConfig);
  }
  public double getGripperPosition(){
    double gripperPosition = gripper.getPosition().getValueAsDouble() * (2d * Math.PI);
    SmartDashboard.putNumber("Gripper pose", gripperPosition);
    return gripperPosition;
  } 
  public void setGripperPosition(double position) {
    position /= (2d*Math.PI);
    gripper.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));
  }
  public void openGripper(){
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperOpen);
    gripper.getConfigurator().apply(gripperConfig);
    gripper.set(ArmConstants.gripperPowerOpen);

  }
  public void closeGripper(){
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperClose);
    gripper.getConfigurator().apply(gripperConfig);
    gripper.set(ArmConstants.gripperPowerClose);
  }
  public void stopGripper() {
    gripper.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
