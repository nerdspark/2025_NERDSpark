// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;
import frc.robot.Constants.ArmSetPoints;

public class Arm extends SubsystemBase {

  private TalonFX shoulderLeft, shoulderRight, elbowLeft, elbowRight, wristFlip, wristTwist;

  /** Creates a new Arm. */
  public Arm() {

    shoulderLeft = new TalonFX(ArmConstants.shoulderMotorLeftPort, "rio"); 
    shoulderRight = new TalonFX(ArmConstants.shoulderMotorRightPort, "rio"); 
    elbowLeft = new TalonFX(ArmConstants.elbowMotorLeftPort, "rio");
    elbowRight = new TalonFX(ArmConstants.elbowMotorRightPort, "rio");
    wristFlip = new TalonFX(ArmConstants.wristMotorPort, "rio");
    wristTwist = new TalonFX(ArmConstants.handMotorPort, "rio");
    TalonFXConfiguration shoulderconfig = new TalonFXConfiguration();
    TalonFXConfiguration elbowconfig = new TalonFXConfiguration();

    shoulderconfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ArmConstants.currentLimitShoulder)
        .withStatorCurrentLimitEnable(true);
    shoulderconfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(ArmConstants.shoulderOffset)
        .withSensorToMechanismRatio(ArmConstants.shoulderRadPerRot);
    shoulderconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.3);
    shoulderconfig.Slot0 = new Slot0Configs()
        .withKP(ArmGains.shoulderP)
        .withKI(ArmGains.shoulderI)
        .withKD(ArmGains.shoulderD)
        .withKG(ArmGains.shoulderG)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    shoulderLeft
      .getConfigurator()
      .apply(shoulderconfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast)));
    
    shoulderRight
      .getConfigurator()
      .apply(shoulderconfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast)));


    elbowconfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(ArmConstants.currentLimitElbow)
      .withStatorCurrentLimitEnable(true);
    elbowconfig.Feedback = new FeedbackConfigs()
      .withFeedbackRotorOffset(ArmConstants.elbowOffset)
      .withSensorToMechanismRatio(ArmConstants.elbowRadPerRot);
    elbowconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);
    elbowconfig.Slot0 = new Slot0Configs()
      .withKP(ArmGains.elbowP)
      .withKI(ArmGains.elbowI)
      .withKD(ArmGains.elbowD)
      .withKG(ArmGains.elbowG)
      .withGravityType(GravityTypeValue.Arm_Cosine);
        
    elbowLeft
      .getConfigurator()
      .apply(elbowconfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)));

    elbowRight
      .getConfigurator()
      .apply(elbowconfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)));

          elbowRight.setPosition(Constants.ArmConstants.elbowOffset);
          elbowLeft.setPosition(Constants.ArmConstants.elbowOffset);
          shoulderRight.setPosition(Constants.ArmConstants.shoulderOffset);
          shoulderLeft.setPosition(Constants.ArmConstants.shoulderOffset);

    // shoulderconfig.CurrentLimits = new CurrentLimitsConfigs()
    //     .withStatorCurrentLimit(ArmConstants.currentLimitShoulder)
    //     .withStatorCurrentLimitEnable(true);
    // shoulderconfig.Feedback = new FeedbackConfigs()
    //     .withFeedbackRotorOffset(0.0) // ArmConstants.shoulderOffset)
    //     .withSensorToMechanismRatio(ArmConstants.shoulderRadPerRot);
    // shoulderconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.3);
    // shoulderconfig.Slot0 = new Slot0Configs()
    //     .withKP(ArmGains.shoulderP)
    //     .withKI(ArmGains.shoulderI)
    //     .withKD(ArmGains.shoulderD)
    //     .withKG(ArmGains.shoulderG)
    //     .withGravityType(GravityTypeValue.Arm_Cosine);

    // shoulder
    //   .getConfigurator()
    //   .apply(shoulderconfig.withMotorOutput(new MotorOutputConfigs()
    //     .withInverted(InvertedValue.Clockwise_Positive)
    //     .withNeutralMode(NeutralModeValue.Brake)));


    // elbowconfig.CurrentLimits = new CurrentLimitsConfigs()
    //   .withStatorCurrentLimit(ArmConstants.currentLimitElbow)
    //   .withStatorCurrentLimitEnable(true);
    // elbowconfig.Feedback = new FeedbackConfigs()
    //   .withFeedbackRotorOffset(0.0) // ArmConstants.elbowOffset)
    //   .withSensorToMechanismRatio(ArmConstants.elbowRadPerRot);
    // elbowconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);
    // elbowconfig.Slot0 = new Slot0Configs()
    //   .withKP(ArmGains.elbowP)
    //   .withKI(ArmGains.elbowI)
    //   .withKD(ArmGains.elbowD)
    //   .withKG(ArmGains.elbowG)
    //   .withGravityType(GravityTypeValue.Arm_Cosine);
        
    // elbow
    //   .getConfigurator()
    //   .apply(elbowconfig.withMotorOutput(new MotorOutputConfigs()
    //     .withInverted(InvertedValue.Clockwise_Positive)
    //       .withNeutralMode(NeutralModeValue.Brake)));

  }

  public Translation2d getArmPosition() {
    Translation2d jointPos = new Translation2d(
                Math.cos(getShoulderLeftPosition()) * ArmConstants.baseStageLength,
                Math.sin(getShoulderLeftPosition()) * ArmConstants.baseStageLength);
        Translation2d jointToEndPos = new Translation2d(
                Math.cos(getElbowLeftPosition()) * ArmConstants.secondStageLength,
                Math.sin(getElbowLeftPosition()) * ArmConstants.secondStageLength);
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
        SmartDashboard.putNumber("shoulderLeftPosition error", shoulderPosition - getShoulderLeftPosition());
        SmartDashboard.putNumber("shoulderRightPosition error", shoulderPosition - getShoulderRightPosition());
        SmartDashboard.putNumber("elbow Left Position error", elbowPosition - getElbowLeftPosition());
        SmartDashboard.putNumber("elbow Right Position error", elbowPosition - getElbowRightPosition());
  }


  public void setElbowPosition(double position) {
    // position -= getShoulderLeftPosition() * (1.0 - ArmConstants.virtual4BarGearRatio);
    position /= (2d * Math.PI);

    // SmartDashboard.putNumber("elbow position set raw", position);
    // if ((Math.abs(position - elbowLeft.getPosition().getValueAsDouble()) + Math.abs(position -
    // elbowRight.getPosition().getValueAsDouble())) < 0.14 && (Math.abs(position -
    // (ArmConstants.elbowOffset/Math.PI/2.0)) < 0.01)) {
    //     elbowLeft.setControl(new DutyCycleOut(0));
    //     elbowRight.setControl(new DutyCycleOut(0));
    // } else {
    elbowLeft.setControl(
            new PositionVoltage(position).withPosition(position));
    elbowRight.setControl(
            new PositionVoltage(position).withPosition(position));
    // }
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

  public double getElbowLeftPosition() {
    double elbowPose = elbowLeft.getPosition().getValueAsDouble() * (2d * Math.PI);
    // elbowPose += getShoulderLeftPosition() * (1.0 - ArmConstants.virtual4BarGearRatio);
    SmartDashboard.putNumber("elbow l position", elbowPose);
    // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeft.getPosition()*24.0/42.0);
    // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeft.getPosition());
    return elbowPose;
    //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
}

public double getElbowRightPosition() {
    double elbowPose = elbowRight.getPosition().getValueAsDouble() * (2d * Math.PI);
    // elbowPose += getShoulderRightPosition() * (1.0 - ArmConstants.virtual4BarGearRatio);
    SmartDashboard.putNumber("elbow r position", elbowPose);
    // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeft.getPosition()*24.0/42.0);
    // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeft.getPosition());
    return elbowPose;
    //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
}

   public void setShoulderPosition(double position) {
        position /= (2d * Math.PI);

        position = MathUtil.clamp(position, -0.1, 2.5);

        // SmartDashboard.putNumber("shoulder position set raw", position);
        if ((Math.abs(position - shoulderLeft.getPosition().getValueAsDouble())
                                + Math.abs(
                                        position - shoulderRight.getPosition().getValueAsDouble()))
                        < 0.1
                && Math.abs(position - (ArmConstants.shoulderOffset / Math.PI / 2.0)) < 0.01) {
            shoulderLeft.setControl(new DutyCycleOut(0));
            shoulderRight.setControl(new DutyCycleOut(0));
        } else {
            shoulderLeft.setControl(
                    new PositionVoltage(position).withFeedForward(position).withPosition(position));
            shoulderRight.setControl(
                    new PositionVoltage(position).withFeedForward(position).withPosition(position));
        }
    }


  // set the velocity of each joint separately: 
  public double getShoulderLeftPosition() {
    double position = shoulderLeft.getPosition().getValueAsDouble() * (2d * Math.PI);
    SmartDashboard.putNumber("shoulder l position", position);
    return position;
}

  public double getShoulderRightPosition() {
    double position = shoulderRight.getPosition().getValueAsDouble() * (2d * Math.PI);
    SmartDashboard.putNumber("shoulder r position", position);
    return position;
    
  }

  public void setShouldervelocity(double velocity) {
    shoulderLeft.setControl(new VelocityVoltage(velocity));
    shoulderRight.setControl(new VelocityVoltage(velocity));
  }

  public void setElbowVelocity(double velocity) {
    elbowLeft.setControl(new VelocityVoltage(velocity));
    elbowRight.setControl(new VelocityVoltage(velocity));
  }

  public void setWristFlipVelocity(double velocity) {
    wristFlip.setControl(new VelocityVoltage(velocity));
  }

  public void setWristTwistVelocity(double velocity) {
    wristTwist.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void periodic() {
    getShoulderLeftPosition();
    getShoulderRightPosition();
    getElbowLeftPosition();
    getElbowRightPosition();
    SmartDashboard.putNumber("left elbow amp", elbowLeft.getDutyCycle().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
