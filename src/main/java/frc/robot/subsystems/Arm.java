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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmGains;
import frc.robot.Constants.ArmSetPoints;

public class Arm extends SubsystemBase {

  private TalonFX shoulder;
  private TalonFX elbow;
  private TalonFX wristFlip;
  private TalonFX wristTwist; //TODO find a better name for this

  /** Creates a new Arm. */
  public Arm() {

    shoulder = new TalonFX(ArmConstants.shoulderMotorPort, "canivore1"); 
    elbow = new TalonFX(ArmConstants.elbowMotorPort, "canivore1");
    wristFlip = new TalonFX(ArmConstants.wristMotorPort, "canivore1");
    wristTwist = new TalonFX(ArmConstants.handMotorPort, "canivore1");

    TalonFXConfiguration shoulderconfig = new TalonFXConfiguration();
    TalonFXConfiguration elbowconfig = new TalonFXConfiguration();

    shoulderconfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ArmConstants.currentLimitShoulder)
        .withStatorCurrentLimitEnable(true);
    shoulderconfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(0.0) // ArmConstants.shoulderOffset)
        .withSensorToMechanismRatio(ArmConstants.shoulderRadPerRot);
    shoulderconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.3);
    shoulderconfig.Slot0 = new Slot0Configs()
        .withKP(ArmGains.shoulderP)
        .withKI(ArmGains.shoulderI)
        .withKD(ArmGains.shoulderD)
        .withKG(ArmGains.shoulderG)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    shoulder
      .getConfigurator()
      .apply(shoulderconfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)));


    elbowconfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(ArmConstants.currentLimitElbow)
      .withStatorCurrentLimitEnable(true);
    elbowconfig.Feedback = new FeedbackConfigs()
      .withFeedbackRotorOffset(0.0) // ArmConstants.elbowOffset)
      .withSensorToMechanismRatio(ArmConstants.elbowRadPerRot);
    elbowconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);
    elbowconfig.Slot0 = new Slot0Configs()
      .withKP(ArmGains.elbowP)
      .withKI(ArmGains.elbowI)
      .withKD(ArmGains.elbowD)
      .withKG(ArmGains.elbowG)
      .withGravityType(GravityTypeValue.Arm_Cosine);
        
    elbow
      .getConfigurator()
      .apply(elbowconfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake)));

  }

  public Translation2d getArmPosition() {
    Translation2d jointPos = new Translation2d(
                Math.cos(getShoulderPosition()) * ArmConstants.baseStageLength,
                Math.sin(getShoulderPosition()) * ArmConstants.baseStageLength);
        Translation2d jointToEndPos = new Translation2d(
                Math.cos(getElbowPosition()) * ArmConstants.secondStageLength,
                Math.sin(getElbowPosition()) * ArmConstants.secondStageLength);
        SmartDashboard.putNumber("arm x position", jointPos.plus(jointToEndPos).getX());
        SmartDashboard.putNumber("arm y position", jointPos.plus(jointToEndPos).getY());
        return jointPos.plus(jointToEndPos);
  }

  public void setArmPosition(Translation2d position, boolean inBend) {  // rotates the base two stages 

    double distance = MathUtil.clamp(position.getNorm(), ArmSetPoints.home.getNorm(), ArmConstants.baseStageLength + ArmConstants.secondStageLength);

        double BaseAngleArmDiff = Math.acos(((distance * distance)
                        + (ArmConstants.baseStageLength * ArmConstants.baseStageLength)
                        - (ArmConstants.secondStageLength * ArmConstants.secondStageLength))
                / (2 * distance * ArmConstants.baseStageLength));
        double SecondAngleArmDiff = Math.acos(((distance * distance)
                        - (ArmConstants.baseStageLength * ArmConstants.baseStageLength)
                        + (ArmConstants.secondStageLength * ArmConstants.secondStageLength))
                / (2 * distance * ArmConstants.secondStageLength));
        double shoulderPosition = position.getAngle().getRadians() + (BaseAngleArmDiff * (inBend ? 1 : -1));
        double elbowPosition = position.getAngle().getRadians() + (SecondAngleArmDiff * (inBend ? -1 : 1));
        setShoulderPosition(shoulderPosition);
        setElbowPosition(elbowPosition);

        // smart dashboard
        SmartDashboard.putNumber("shouldertarget", shoulderPosition);
        SmartDashboard.putNumber("elbowtarget", elbowPosition);
        SmartDashboard.putNumber("shoulderPosition error", shoulderPosition - getShoulderPosition());
        SmartDashboard.putNumber("elbow Position error", elbowPosition - getElbowPosition());
  }

  public double getShoulderPosition() {
    SmartDashboard.putNumber("shoulder l position raw", shoulder.getPosition().getValueAsDouble());
    return shoulder.getPosition().getValueAsDouble() * (2d * Math.PI);
  }

  public double getElbowPosition() {
    double elbowPose = elbow.getPosition().getValueAsDouble() * (2d * Math.PI);
    SmartDashboard.putNumber("elbow l position raw", elbow.getPosition().getValueAsDouble());
    return elbowPose;
  }

  public void setWristTwistPosition(double position) {
    position /= (2d*Math.PI);

    SmartDashboard.putNumber("hand position set raw", position);
    wristTwist.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));

  }

  public void setWristFlipPosition(double position) {
    position /= (2d*Math.PI);

    SmartDashboard.putNumber("hand position set raw", position);
    wristFlip.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));
  } 

  public void setElbowPosition(double position) {
    // position -= getShoulderLeftPosition() * (1.0 - ArmConstants.virtual4BarGearRatio);
    position /= (2d * Math.PI);

    SmartDashboard.putNumber("elbow position set raw", position);
    elbow.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));
  }

  public void setShoulderPosition(double position) {

    position /= (2d * Math.PI);

    position = MathUtil.clamp(position, -0.1, 2.5);

    SmartDashboard.putNumber("shoulder position set raw", position);
    shoulder.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));

  }


  // set the velocity of each joint separately: 

  public void setShoulderVelocity(double velocity) {
    shoulder.setControl(new VelocityVoltage(velocity));
  }

  public void setElbowVelocity(double velocity) {
    elbow.setControl(new VelocityVoltage(velocity));
  }

  public void setWristFlipVelocity(double velocity) {
    wristFlip.setControl(new VelocityVoltage(velocity));
  }

  public void setWristTwistVelocity(double velocity) {
    wristTwist.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
