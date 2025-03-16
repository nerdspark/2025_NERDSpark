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
  private CANrange sensorMiddle;
  private CANrange sensorLeft;
  private CANrange sensorRight;
  private double distanceToTrip = 0.15;
  private double distanceToTripMiddle = 0.2;

  /** Creates a new Gripper. */
  public Gripper() {
    gripper = new TalonFX(ArmConstants.gripperMotorPort, ArmConstants.armCanBus);
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
            .withNeutralMode(NeutralModeValue.Brake);
        gripper
          .getConfigurator()
          .apply(gripperConfig);
    sensorMiddle = new CANrange(ArmConstants.rangeMiddlePort, ArmConstants.armCanBus);
    sensorLeft = new CANrange(ArmConstants.rangeLeftPort, ArmConstants.armCanBus);
    sensorRight = new CANrange(ArmConstants.rangeRightPort, ArmConstants.armCanBus);
    CANrangeConfiguration sensorConfig = new CANrangeConfiguration();
    CANrangeConfiguration sensorMiddleConfig = new CANrangeConfiguration();
    sensorConfig.ProximityParams = new ProximityParamsConfigs().withProximityThreshold(distanceToTrip);
    sensorMiddleConfig.ProximityParams = new ProximityParamsConfigs().withProximityThreshold(distanceToTripMiddle);
    sensorMiddle.getConfigurator().apply(sensorMiddleConfig);
    sensorLeft.getConfigurator().apply(sensorConfig);
    sensorRight.getConfigurator().apply(sensorConfig);
    gripper.setPosition(0);
  }
  public double getGripperPosition(){
    double gripperPosition = gripper.getPosition().getValueAsDouble() * (2d * Math.PI);
    // SmartDashboard.putNumber("Gripper pose", gripperPosition);
    return gripperPosition;
  } 
  public void setGripperPosition(double position) {
    // position /= (2d*Math.PI);
    gripper.setControl(new PositionVoltage(position).withPosition(position));
  }
  public void openGripper(){
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperOpen);
    gripper.getConfigurator().apply(gripperConfig);
    gripper.set(ArmConstants.gripperPowerOpen);
  }
  public void openGripperStrong(){
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperClose);
    gripper.getConfigurator().apply(gripperConfig);
    gripper.set(-ArmConstants.gripperPowerClose);
  }
  public void setCurrentLimitStrong(){
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperClose);
          gripper.getConfigurator().apply(gripperConfig);

  }
  public void setCurrentLimitWeak(){
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperOpen);
          gripper.getConfigurator().apply(gripperConfig);

  }
  public void closeGripper(){
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperClose);
    gripper.getConfigurator().apply(gripperConfig);
    gripper.set(ArmConstants.gripperPowerClose);
  }
  public void closeGripperWeak(){
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.currentLimitGripperOpen);
    gripper.getConfigurator().apply(gripperConfig);
    gripper.set(-ArmConstants.gripperPowerOpen);
  }
  public void stopGripper() {
    gripper.stopMotor();
  }
  public double getRangeRightDistance() {
    return sensorRight.getDistance().getValueAsDouble();
  }
  public double getRangeLeftDistance() {
    return sensorLeft.getDistance().getValueAsDouble();
  }
  public double getRangeMiddleDistance() {
    return sensorMiddle.getDistance().getValueAsDouble();
  }
  /** teleop */
  public boolean getMiddleDetected(){
    return sensorMiddle.getIsDetected().getValue();
  }
  /** teleop */
  public boolean getLeftDetected(){
    return sensorLeft.getIsDetected().getValue();
  }
  /** teleop */
  public boolean getRightDetected(){
    return sensorRight.getIsDetected().getValue();
  }
  public boolean getMiddleToTrip(){
    return sensorMiddle.getIsDetected().getValue() && getRangeMiddleDistance()<distanceToTripMiddle  && getRangeMiddleDistance() > 0.01;
  }
  public boolean getRightToTrip(){
    return sensorRight.getIsDetected().getValue() && getRangeRightDistance()<distanceToTrip  && getRangeRightDistance() > 0.01;
  }
  public boolean getLeftToTrip(){
    return sensorLeft.getIsDetected().getValue() && getRangeLeftDistance()<distanceToTrip  && getRangeLeftDistance() > 0.01;
  }
  /** auton */
  public boolean getDetected(){
    // return Math.min(Math.min(getRangeMiddleDistance(), getRangeLeftDistance()), getRangeRightDistance()) < 0.1;
    return (getLeftToTrip() || getRightToTrip() || getMiddleToTrip());
  }
  public boolean getShelfDetected(){
    // return Math.min(Math.min(getRangeMiddleDistance(), getRangeLeftDistance()), getRangeRightDistance()) < 0.1;
    return (getDetected());
  }
  public boolean getFunnelDetected(){
    // return Math.min(Math.min(getRangeMiddleDistance(), getRangeLeftDistance()), getRangeRightDistance()) < 0.1;
    return (getRangeMiddleDistance() < 0.26);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("middle sensor", getRangeMiddleDistance());
    SmartDashboard.putNumber("l sensor", (getRangeLeftDistance()));
    SmartDashboard.putBoolean("middle det", getMiddleDetected());
    SmartDashboard.putNumber("gripper pos", gripper.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber(getName(), distanceToTrip)
    // This method will be called once per scheduler run
  }
}
