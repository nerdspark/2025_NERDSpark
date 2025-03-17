// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.io.Console;
import java.util.ArrayList;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ArmVelocityGains;
import frc.robot.util.ArmPoint;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;

public class Arm extends SubsystemBase {

  private TalonFX shoulderLeft, shoulderRight, elbowLeft, elbowRight, wrist;
  
  public boolean finishedMoving = false;
  private boolean wristStopped = false;
  private TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
  public boolean stowing = false;
  private SlewRateLimiter shoulderLimiter = new SlewRateLimiter(ArmConstants.shoulderSlewRate);
  private SlewRateLimiter elbowLimiter = new SlewRateLimiter(ArmConstants.elbowSlewRate);


  public double wristTarget = 0.0;
  /** Creates a new Arm. */
  public Arm() {
    
    shoulderLeft = new TalonFX(ArmConstants.shoulderMotorLeftPort, ArmConstants.armCanBus); 
    shoulderRight = new TalonFX(ArmConstants.shoulderMotorRightPort, ArmConstants.armCanBus); 
    elbowLeft = new TalonFX(ArmConstants.elbowMotorLeftPort, ArmConstants.armCanBus);
    elbowRight = new TalonFX(ArmConstants.elbowMotorRightPort, ArmConstants.armCanBus);
    wrist = new TalonFX(ArmConstants.wristMotorPort, ArmConstants.armCanBus);
    
    TalonFXConfiguration elbowConfig = new TalonFXConfiguration();
    TalonFXConfiguration wristTwistConfig = new TalonFXConfiguration();
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    shoulderConfig.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(ArmConstants.currentLimitShoulder)
        .withStatorCurrentLimitEnable(true);
    shoulderConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(ArmConstants.shoulderOffset)
        .withSensorToMechanismRatio(ArmConstants.shoulderRadPerRot);
    shoulderConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.03);
    shoulderConfig.Slot0 = new Slot0Configs()
        .withKP(ArmGains.shoulderP)
        .withKI(ArmGains.shoulderI)
        .withKD(ArmGains.shoulderD)
        .withKG(ArmGains.shoulderG)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    shoulderConfig.Slot1 = new Slot1Configs()
        .withKP(ArmVelocityGains.shoulderP)
        .withKI(ArmVelocityGains.shoulderI)
        .withKD(ArmVelocityGains.shoulderD)
        .withKG(ArmVelocityGains.shoulderG)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    

    shoulderLeft
      .getConfigurator()
      .apply(shoulderConfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)));
    
    shoulderRight
      .getConfigurator()
      .apply(shoulderConfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)));


    elbowConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(ArmConstants.currentLimitElbow)
      .withStatorCurrentLimitEnable(true);
    elbowConfig.Feedback = new FeedbackConfigs()
      .withFeedbackRotorOffset(ArmConstants.elbowOffset)
      .withSensorToMechanismRatio(ArmConstants.elbowRadPerRot);
    elbowConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.03);
    elbowConfig.Slot0 = new Slot0Configs()
      .withKP(ArmGains.elbowP)
      .withKI(ArmGains.elbowI)
      .withKD(ArmGains.elbowD)
      .withKG(ArmGains.elbowG)
      .withGravityType(GravityTypeValue.Arm_Cosine);
    elbowConfig.Slot1 = new Slot1Configs()
      .withKP(ArmVelocityGains.elbowP)
      .withKI(ArmVelocityGains.elbowI)
      .withKD(ArmVelocityGains.elbowD)
      .withKG(ArmVelocityGains.elbowG)
      .withGravityType(GravityTypeValue.Arm_Cosine);
        
    elbowLeft
      .getConfigurator()
      .apply(elbowConfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake)));

    elbowRight
      .getConfigurator()
      .apply(elbowConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake)));

          
    
    
    
    wristConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(ArmConstants.currentLimitWrist)
      .withStatorCurrentLimitEnable(true);
    wristConfig.Feedback = new FeedbackConfigs()
      .withFeedbackRotorOffset(ArmConstants.wristOffset)
      .withSensorToMechanismRatio(ArmConstants.wristRadPerRot);
    wristConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.05);
    wristConfig.Slot0 = new Slot0Configs()
      .withKP(ArmGains.wristP)
      .withKI(ArmGains.wristI)
      .withKD(ArmGains.wristD);
    wristConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(ArmGains.wristAcceleration).withMotionMagicCruiseVelocity(ArmGains.wristVelocity);
    wrist
      .getConfigurator()
      .apply(wristConfig.withMotorOutput(new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)));
    
    
    resetOffsets();
  }
  public void resetVelocityLimiters() {
    shoulderLimiter.reset(getShoulderVelocity());
    elbowLimiter.reset(getElbowVelocity());
  }
  public double getShoulderVelocity() {
    return (shoulderLeft.getVelocity().getValueAsDouble() + shoulderRight.getVelocity().getValueAsDouble())/2;
  }
  public double getElbowVelocity() {
    return (elbowLeft.getVelocity().getValueAsDouble() + elbowRight.getVelocity().getValueAsDouble())/2;
  }
  // public void setBrakeMode(boolean BrakeModeEnabled) {
  //   if (BrakeModeEnabled) {
  //     shoulderLeft
  //       .getConfigurator()
  //       .apply(shoulderConfig.withMotorOutput(new MotorOutputConfigs()
  //         .withInverted(InvertedValue.CounterClockwise_Positive)
  //         .withNeutralMode(NeutralModeValue.Brake)));
  //     shoulderRight
  //       .getConfigurator()
  //       .apply(shoulderConfig.withMotorOutput(new MotorOutputConfigs()
  //         .withInverted(InvertedValue.Clockwise_Positive)
  //         .withNeutralMode(NeutralModeValue.Brake)));
  //   } else {
  //     shoulderLeft
  //       .getConfigurator()
  //       .apply(shoulderConfig.withMotorOutput(new MotorOutputConfigs()
  //         .withInverted(InvertedValue.CounterClockwise_Positive)
  //         .withNeutralMode(NeutralModeValue.Coast)));
  //     shoulderRight
  //       .getConfigurator()
  //       .apply(shoulderConfig.withMotorOutput(new MotorOutputConfigs()
  //         .withInverted(InvertedValue.Clockwise_Positive)
  //         .withNeutralMode(NeutralModeValue.Coast)));
  //   }
  // }
  public void resetOffsets() {
    elbowRight.setPosition(Constants.ArmConstants.elbowOffset);
    elbowLeft.setPosition(Constants.ArmConstants.elbowOffset);
    shoulderRight.setPosition(Constants.ArmConstants.shoulderOffset);
    shoulderLeft.setPosition(Constants.ArmConstants.shoulderOffset);
    wrist.setPosition(Constants.ArmConstants.wristOffset);
  }

  public Translation2d getArmPosition() {
    Translation2d jointPos = new Translation2d(
                Math.cos(getShoulderPosition()) * ArmConstants.baseStageLength,
                Math.sin(getShoulderPosition()) * ArmConstants.baseStageLength);
        Translation2d jointToEndPos = new Translation2d(
                Math.cos(getElbowPosition()) * ArmConstants.secondStageLength,
                Math.sin(getElbowPosition()) * ArmConstants.secondStageLength);
        // SmartDashboard.putNumber("arm x position", jointPos.plus(jointToEndPos).getX());
        // SmartDashboard.putNumber("arm y position", jointPos.plus(jointToEndPos).getY());
        return jointPos.plus(jointToEndPos);
        // return ArmSetpoints.armSetPoints[4].position;
  }
  public ArmPoint getArmState() {
    return new ArmPoint(getArmPosition(), getCurrentInBend(), getWristTarget());
  }

  public void setArmPosition(Translation2d position, boolean inBend) {  // rotates the two base stages 

    double distance = MathUtil.clamp(position.getNorm(), ArmSetpoints.home.getNorm(), ArmConstants.baseStageLength + ArmConstants.secondStageLength);

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

        resetVelocityLimiters();
        // smart dashboard
        // SmartDashboard.putNumber("shouldertarget", shoulderPosition);
        // SmartDashboard.putNumber("elbowtarget", elbowPosition);
        // SmartDashboard.putNumber("shoulderLeftPosition error", shoulderPosition - getShoulderPosition());
        // SmartDashboard.putNumber("elbow Left Position error", elbowPosition - getElbowPosition());
  }
  public void setVelocity(Translation2d velocity, boolean inBend){
    Translation2d position = getArmPosition().plus(velocity.times(ArmConstants.linearApproximationTime));
    // SmartDashboard.putNumber("velocity x", velocity.getX());
    // SmartDashboard.putNumber("velocity y", velocity.getY());
    double distance = MathUtil.clamp(position.getNorm(), ArmSetpoints.home.getNorm(), ArmConstants.baseStageLength + ArmConstants.secondStageLength);
    double currentMaxV = 30;

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
    double shoulderVelocity = (shoulderPosition - getShoulderPosition())/ArmConstants.linearApproximationTime;
    double elbowVelocity =(elbowPosition - getElbowPosition())/ArmConstants.linearApproximationTime;
    // SmartDashboard.putNumber("shoulder target velocity", shoulderVelocity);
    // SmartDashboard.putNumber("elbow target velocity", elbowVelocity);
    // SmartDashboard.putNumber("target pos x", position.getX());
    // SmartDashboard.putNumber("target pos y", position.getY());
    currentMaxV = Math.max(Math.abs(shoulderVelocity), Math.abs(elbowVelocity));
    // SmartDashboard.putNumber("current max v", currentMaxV);
    // if(currentMaxV > ArmConstants.maxMotorVelocity){
    //   shoulderVelocity = shoulderVelocity * (ArmConstants.maxMotorVelocity/currentMaxV);
    //   elbowVelocity = elbowVelocity * (ArmConstants.maxMotorVelocity/currentMaxV);
    // }
    // if(shoulderPosition < ArmConstants.shoulderOffset *(2*Math.PI)){
    //   // SmartDashboard.putBoolean("shoulder soft limit triggered", true);
      
    //   setShoulderVelocity(0);
    //   shoulderLimiter.reset(0);
    // } else {
      // SmartDashboard.putBoolean("shoulder soft limit triggered", false);
      setShoulderVelocity(shoulderLimiter.calculate(shoulderVelocity));
    // }
    // if(Math.abs(elbowPosition - shoulderPosition) > Math.abs(ArmConstants.elbowOffset - ArmConstants.shoulderOffset) * (2*Math.PI)){
      // SmartDashboard.putBoolean("elbow soft limit triggered", true);
    //   setElbowVelocity((shoulderVelocity));
    //   elbowLimiter.reset(shoulderVelocity);
    // } else {
      // SmartDashboard.putBoolean("elbow soft limit triggered", false);
      setElbowVelocity(elbowLimiter.calculate(elbowVelocity));
    // }  
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
            new PositionVoltage(position).withPosition(position).withSlot(0));
    elbowRight.setControl(
            new PositionVoltage(position).withPosition(position).withSlot(0));
    // }
    // SmartDashboard.putNumber("elbow Left Position error", position - elbowLeft.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("elbow Right Position error", position - elbowRight.getPosition().getValueAsDouble());

}
  public void stopWrist() {
    wristStopped = true;
  }
  public double getWristTarget(){
    double wristPosition = (wrist.getPosition().getValueAsDouble() * (2d * Math.PI));
    wristPosition -= getElbowPosition() * (ArmConstants.wristToElbowRatio - 1.0);
    // SmartDashboard.putNumber("wrist flip position", wristPosition);
    return wristPosition;
  }

  public void setWristTarget(double position) {
    wristTarget = position;
    wristStopped = false;
    // wristFinishedMoving = false;
// ratio = oo: wrist change amount -oo
// ratio = 1: wrist change amount 0
// ratio = 0: wrist change amount 1
    // SmartDashboard.putNumber("wrist flip position set raw", position);

  }
  public void updateWristSetpoints() {
    if (wristStopped) {
      wrist.stopMotor();
    } else {
      double flipPosition = wristTarget;
      flipPosition += getElbowPosition() * (ArmConstants.wristToElbowRatio - 1.0);
      flipPosition /= (2d*Math.PI);
      wrist.setControl(new MotionMagicVoltage(flipPosition).withPosition(flipPosition).withSlot(0));
    }

  }
  
  public double getElbowPosition() {
    double elbowPose = (elbowLeft.getPosition().getValueAsDouble() + elbowRight.getPosition().getValueAsDouble())/2 * (2d * Math.PI);
    // SmartDashboard.putNumber("elbow position", elbowPose);
    // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeft.getPosition()*24.0/42.0);
    // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeft.getPosition());
    return elbowPose;
    //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
}
public void setShoulderPower(double power) {
  shoulderLeft.set(-power);
  shoulderRight.set(power);
}
public void setShoulderAmpLimit(double amplimit) {
  
  shoulderConfig.CurrentLimits = new CurrentLimitsConfigs()
  .withStatorCurrentLimit(amplimit);
  shoulderLeft.getConfigurator().apply(shoulderConfig);
  shoulderRight.getConfigurator().apply(shoulderConfig);
}


   public void setShoulderPosition(double position) {
        position /= (2d * Math.PI);

        //position = MathUtil.clamp(position, -0.1, 2.5);

        // SmartDashboard.putNumber("shoulder position set raw", position);
        // if ((Math.abs(position - shoulderLeft.getPosition().getValueAsDouble())
        //                         + Math.abs(
        //                                 position - shoulderRight.getPosition().getValueAsDouble()))
        //                 < 0.1
        //         && Math.abs(position - (ArmConstants.shoulderOffset / Math.PI / 2.0)) < 0.01) {
        //     shoulderLeft.setControl(new DutyCycleOut(0));
        //     shoulderRight.setControl(new DutyCycleOut(0));
        // } else {
            shoulderLeft.setControl(
                    new PositionVoltage(position).withPosition(position).withSlot(0));
            shoulderRight.setControl(
                    new PositionVoltage(position).withPosition(position).withSlot(0));
            // SmartDashboard.putNumber("shoulder target pos", position);
            // SmartDashboard.putNumber("shoulder Left Position error", position - shoulderLeft.getPosition().getValueAsDouble());
            // SmartDashboard.putNumber("shoulder Right Position error", position - shoulderRight.getPosition().getValueAsDouble());
        //}
    }


  // set the velocity of each joint separately: 
  public double getShoulderPosition() {
    
    double position = (shoulderLeft.getPosition().getValueAsDouble() + shoulderRight.getPosition().getValueAsDouble())/2 * (2d * Math.PI);
    // SmartDashboard.putNumber("shoulder position", position);

    return position;
}


  public void setShoulderVelocity(double velocity) {
    shoulderLeft.setControl(new VelocityVoltage(velocity).withSlot(1));
    shoulderRight.setControl(new VelocityVoltage(velocity).withSlot(1));
    // SmartDashboard.putNumber("shoulder target velocity limited", velocity);
  }

  public void setElbowVelocity(double velocity) {
    elbowLeft.setControl(new VelocityVoltage(velocity).withSlot(1));
    elbowRight.setControl(new VelocityVoltage(velocity).withSlot(1));
    // SmartDashboard.putNumber("elbow target velocity limited", velocity);
  }

  public void setWristVelocity(double velocity) {
    wrist.setControl(new VelocityVoltage(velocity));
  }


  public boolean getCurrentInBend() {
    return getElbowPosition() - getShoulderPosition() < 0;
  }

  public void stopArm() {
    elbowLeft.stopMotor();
    elbowRight.stopMotor();
    shoulderLeft.stopMotor();
    shoulderRight.stopMotor();
  }

  public void setStowing(boolean Stowing) {
    stowing = Stowing;
  }
  public boolean wristFinishedMoving() {
    return Math.abs(getWristTarget() - wristTarget) < 0.2;
  }
  
  @Override
  public void periodic() {
    updateWristSetpoints();
    // getShoulderPosition();
    // getElbowPosition();
    
    // SignalLogger.writeDouble("shoulder Left amps", shoulderLeft.getStatorCurrent().getValueAsDouble());
    // SignalLogger.writeDouble("shoulder right amps", shoulderRight.getStatorCurrent().getValueAsDouble());
    
    // SmartDashboard.putNumber("shoulder Left amps", shoulderLeft.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("shoulder right amps", shoulderRight.getStatorCurrent().getValueAsDouble());
    // getWristPosition();
    // getWristTwistPosition();
    // SmartDashboard.putNumber("wrist flip output", wrist.getClosedLoopOutput().getValueAsDouble());
    // SmartDashboard.putNumber("wrist flip amp", wrist.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("wrist Twist output", wristTwist.getClosedLoopOutput().getValueAsDouble());
    // SmartDashboard.putNumber("wrist Twist amp", wristTwist.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("wrist twist pos", getWristTwistPosition());
    // SmartDashboard.putNumber("wrist flip pos", getWristPosition());
    SmartDashboard.putNumber("arm pose x", getArmPosition().getX());
    SmartDashboard.putNumber("arm pose y", getArmPosition().getY());
    SmartDashboard.putNumber("elbow pos", getElbowPosition());
    SmartDashboard.putNumber("shoulder pos", getShoulderPosition());
    SmartDashboard.putNumber("wrist flip pos", getWristTarget());
    // SmartDashboard.putNumber("shoulder velocity", getShoulderVelocity());
    // SmartDashboard.putNumber("elbow velocity", getElbowVelocity());
    // SmartDashboard.putNumber("left elbow amp", elbowLeft.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("right shoulder amp", shoulderRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("left shoulder amp", shoulderLeft.getStatorCurrent().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
