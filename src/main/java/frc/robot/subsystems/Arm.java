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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ArmVelocityGains;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmInstantCommand;
import frc.robot.util.ArmPoint;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;

/**
 * Subsystem for the 3-jointed arm with a shoulder, elbow, and wrist.
 *
 * The shoulder and elbow consists of 2 motors each while the wrist consists
 * of a single motor.
 */
public class Arm extends SubsystemBase {

  private TalonFX shoulderLeft, shoulderRight, elbowLeft, elbowRight, wrist;

  public boolean finishedMoving = false;
  private boolean wristStopped = true;
  private TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
  public boolean stowing = false;
  private SlewRateLimiter shoulderLimiter = new SlewRateLimiter(ArmConstants.shoulderSlewRate);
  private SlewRateLimiter elbowLimiter = new SlewRateLimiter(ArmConstants.elbowSlewRate);
  private double wristOffset = ArmConstants.wristOffset;
  private TalonFXConfiguration wristConfig = new TalonFXConfiguration();

  public double wristTarget = ArmSetpoints.homeWrist;
  public double elbowTarget = ArmConstants.shoulderOffset;
  public double shoulderTarget = ArmConstants.elbowOffset;

  /**
   * Create a new Arm.
   */
  public Arm() {
    shoulderLeft = new TalonFX(ArmConstants.shoulderMotorLeftPort, ArmConstants.armCanBus); 
    shoulderRight = new TalonFX(ArmConstants.shoulderMotorRightPort, ArmConstants.armCanBus); 
    elbowLeft = new TalonFX(ArmConstants.elbowMotorLeftPort, ArmConstants.armCanBus);
    elbowRight = new TalonFX(ArmConstants.elbowMotorRightPort, ArmConstants.armCanBus);
    wrist = new TalonFX(ArmConstants.wristMotorPort, ArmConstants.armCanBus);

    TalonFXConfiguration elbowConfig = new TalonFXConfiguration();

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
    shoulderConfig.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(ArmGains.cruiseVelocity)
        .withMotionMagicAcceleration(ArmGains.cruiseAcceleration);
    shoulderLeft
      .getConfigurator()
      .apply(shoulderConfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast)));

    shoulderRight
      .getConfigurator()
      .apply(shoulderConfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast)));


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

    elbowConfig.MotionMagic = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(ArmGains.cruiseVelocity)
      .withMotionMagicAcceleration(ArmGains.cruiseAcceleration);
    elbowLeft
      .getConfigurator()
      .apply(elbowConfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)));

    elbowRight
      .getConfigurator()
      .apply(elbowConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)));

    wristConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(ArmConstants.currentLimitWrist)
      .withStatorCurrentLimitEnable(true);
    wristConfig.Feedback = new FeedbackConfigs()
      .withFeedbackRotorOffset(ArmConstants.wristOffset)
      .withSensorToMechanismRatio(ArmConstants.wristRadPerRot);
    wristConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);
    wristConfig.Slot0 = new Slot0Configs()
      .withKP(ArmGains.wristP)
      .withKI(ArmGains.wristI)
      .withKD(ArmGains.wristD);
    wristConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(ArmGains.wristAcceleration).withMotionMagicCruiseVelocity(ArmGains.wristVelocity);
    wrist
      .getConfigurator()
      .apply(wristConfig.withMotorOutput(new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)));

    resetOffsets();

  }
  public void addToWristOffset(double addTo) {
    // wristOffset += addTo;
    // wrist.getConfigurator().apply(wristConfig.withFeedback(new FeedbackConfigs()
    // .withFeedbackRotorOffset(wristOffset)
    // .withSensorToMechanismRatio(ArmConstants.wristRadPerRot)));
    wrist.setPosition(wrist.getPosition().getValueAsDouble()+addTo);
    // return wristOffset;
  }

  /**
   * Reset velocity limiters for shoulder and elbow.
   */
  public void resetVelocityLimiters() {
    shoulderLimiter.reset(getShoulderVelocity());
    elbowLimiter.reset(getElbowVelocity());
  }

  /**
   * Get shoulder velocity.
   *
   * @return Averaged shoulder velocity in <em>rotations per second</em>.
   */
  public double getShoulderVelocity() {
    return (shoulderLeft.getVelocity().getValueAsDouble() + shoulderRight.getVelocity().getValueAsDouble())/2;
  }

  /**
   * Get elbow velocity.
   *
   * @return Averaged elbow velocity in <em>rotations per second</em>.
   */
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

  /**
   * Set arm to offset positions.
   * The offset positions are
   * <ul>
   * <li> {@link Constants.ArmConstants.elbowOffset}
   * <li> {@link Constants.ArmConstants.shoulderOffset}
   * <li> {@link Constants.ArmConstants.wristOffset}
   * </ul>
   */
  public void resetOffsets() {
    elbowRight.setPosition(Constants.ArmConstants.elbowOffset);
    elbowLeft.setPosition(Constants.ArmConstants.elbowOffset);
    shoulderRight.setPosition(Constants.ArmConstants.shoulderOffset);
    shoulderLeft.setPosition(Constants.ArmConstants.shoulderOffset);
    wrist.setPosition(Constants.ArmConstants.wristOffset);
  }

  /**
   * Obtain the 2d position of the wrist.
   * The 2d plane is the plane in which the arm's movement is constrained.
   *
   * @return The arm position.
   */
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

  /**
   * Get the current state of the arm.
   *
   * @return the current state of the arm.
   */
  public ArmPoint getArmState() {
    return new ArmPoint(getArmPosition(), getCurrentInBend(), getWristPosition());
  }

  /**
   * Move the arm to a specified position by rotating the shoulder and elbow.
   *
   * @param position position to move the wrist to.
   * @param inBend a flag to choose which orientation of the arm to use.
   * * false = bend convex side facing negative rotation direction
   * * true = bend convex side facing positive rotation direction
   */
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
        shoulderTarget = shoulderPosition;
        elbowTarget = elbowPosition;
        setShoulderPosition(shoulderPosition);
        setElbowPosition(elbowPosition);

        resetVelocityLimiters();
        // smart dashboard
        // SmartDashboard.putNumber("shouldertarget", shoulderPosition);
        // SmartDashboard.putNumber("elbowtarget", elbowPosition);
        // SmartDashboard.putNumber("shoulderLeftPosition error", shoulderPosition - getShoulderPosition());
        // SmartDashboard.putNumber("elbow Left Position error", elbowPosition - getElbowPosition());
  }

  /**
   * Set the velocity of the arm by setting the shoulder and elbow velocities.
   *
   * @param velocity vector defining desired velocity of the wrist.
   * @param inBend a flag to choose which orientation of the arm to use.
   */
  public void setVelocity(Translation2d velocity, boolean inBend){
    Translation2d position = getArmPosition().plus(velocity.times(ArmVelocityGains.linearApproximationTime));
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
    double shoulderVelocity = (shoulderPosition - getShoulderPosition())/ArmVelocityGains.linearApproximationTime;
    double elbowVelocity =(elbowPosition - getElbowPosition())/ArmVelocityGains.linearApproximationTime;
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

  /**
   * Request the elbow PIDs to target a position.
   *
   * @param position position to target in <em>radians</em>.
   */
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
    final MotionMagicVoltage m_request = new MotionMagicVoltage(position);
    elbowLeft.setControl(
            m_request.withPosition(position));
    elbowRight.setControl(
            m_request.withPosition(position));
    // }
    // SmartDashboard.putNumber("elbow Left Position error", position - elbowLeft.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("elbow Right Position error", position - elbowRight.getPosition().getValueAsDouble());
  }

  /**
   * Request the wrist to stop moving.
   */
  public void stopWrist() {
    wristStopped = true;
  }

  /**
   * Get the wrist position relative to the wrist joint.
   * This takes into account the belting ratio between the wrist and elbow.
   *
   * @return the wrist position in <em>radians</em>.
   */
  public double getWristPosition(){
    double wristPosition = (wrist.getPosition().getValueAsDouble() * (2d * Math.PI));
    // TODO: Is this subtraction 'safe'? Multiplication can be wrong for
    //       unrestricted angular domains.
    wristPosition -= getElbowPosition() * (ArmConstants.wristToElbowRatio - 1.0);
    // SmartDashboard.putNumber("wrist flip position", wristPosition);
    return wristPosition;
  }

  /**
   * Request the wrist to move to a certain position.
   * This method enables wrist movement if it was requested
   * to stop previously.
   *
   * @param position the wrist position in <em>radians</em>.
   */
  public void setWristTarget(double position) {
    wristTarget = position;
    wristStopped = false;
    // wristFinishedMoving = false;
    // ratio = oo: wrist change amount -oo
    // ratio = 1: wrist change amount 0
    // ratio = 0: wrist change amount 1
    // SmartDashboard.putNumber("wrist flip position set raw", position);
  }

  /**
   * Update wrist to requested movement.
   * If the wrist was requested to stop, its motor will be stopped.
   * Otherwise, the wrist will be set to the last requested target position.
   */
  public void updateWristSetpoints() {
    if (wristStopped) {
      wrist.stopMotor();
    } else {
      double flipPosition = wristTarget;
      // TODO: Same question in getWristTarget()
      flipPosition += getElbowPosition() * (ArmConstants.wristToElbowRatio - 1.0);
      flipPosition /= (2d*Math.PI);
      wrist.setControl(new MotionMagicVoltage(flipPosition).withPosition(flipPosition).withSlot(0));
    }
  }

  /**
   * Get elbow position.
   *
   * @return Averaged elbow position in <em>radians</em>.
   */
  public double getElbowPosition() {
    double elbowPose = (elbowLeft.getPosition().getValueAsDouble() + elbowRight.getPosition().getValueAsDouble())/2 * (2d * Math.PI);
    // SmartDashboard.putNumber("elbow position", elbowPose);
    // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeft.getPosition()*24.0/42.0);
    // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeft.getPosition());
    return elbowPose;
    //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
  }

  public Command goToHome() {
    return new ArmInstantCommand(this, () -> 0);
  }

  /**
   * Set the speed of the shoulder.
   *
   * @param power the speed of the motors from -1 to +1.
   */
  public void setShoulderPower(double power) {
    shoulderLeft.set(-power);
    shoulderRight.set(power);
  }

  /**
   * Set the amp limit of the shoulder motors.
   *
   * @param amplimit the amp limit in <em>Amps</em>.
   */
  public void setShoulderAmpLimit(double amplimit) {
    shoulderConfig.CurrentLimits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(amplimit);
    shoulderLeft.getConfigurator().apply(shoulderConfig);
    shoulderRight.getConfigurator().apply(shoulderConfig);
  }


  /**
   * Request the shoulder PIDs to target a position.
   *
   * @param position position to target in <em>radians</em>.
   */
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
      final MotionMagicVoltage m_request = new MotionMagicVoltage(position);
        shoulderLeft.setControl(
                m_request.withPosition(position));
        shoulderRight.setControl(
                m_request.withPosition(position));
    // SmartDashboard.putNumber("shoulder target pos", position);
    // SmartDashboard.putNumber("shoulder Left Position error", position - shoulderLeft.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("shoulder Right Position error", position - shoulderRight.getPosition().getValueAsDouble());
    //}
  }

  /**
   * Get shoulder position.
   *
   * @return averaged shoulder position in <em>radians</em>.
   */
  public double getShoulderPosition() {
    double position = (shoulderLeft.getPosition().getValueAsDouble() + shoulderRight.getPosition().getValueAsDouble())/2 * (2d * Math.PI);
    // SmartDashboard.putNumber("shoulder position", position);

    return position;
  }

  /**
   * Request the shoulder PIDs to target a velocity.
   *
   * @param velocity velocity to target in <em>rotations per second</em>.
   */
  public void setShoulderVelocity(double velocity) {
    shoulderLeft.setControl(new VelocityVoltage(velocity).withSlot(1));
    shoulderRight.setControl(new VelocityVoltage(velocity).withSlot(1));
    // SmartDashboard.putNumber("shoulder target velocity limited", velocity);
  }

  /**
   * Request the elbow PIDs to target a velocity.
   *
   * @param velocity velocity to target in <em>rotations per second</em>.
   */
  public void setElbowVelocity(double velocity) {
    elbowLeft.setControl(new VelocityVoltage(velocity).withSlot(1));
    elbowRight.setControl(new VelocityVoltage(velocity).withSlot(1));
    // SmartDashboard.putNumber("elbow target velocity limited", velocity);
  }

  /**
   * Request the wrist PID to target a velocity.
   *
   * @param velocity velocity to target in <em>rotations per second</em>.
   */
  public void setWristVelocity(double velocity) {
    wrist.setControl(new VelocityVoltage(velocity));
  }

  /**
   * Get whether the arm is currently bent inwards.
   *
   * @return <code>true</code> if bent inwards.
   * false = bend convex side facing negative rotation direction
   * true = bend convex side facing positive rotation direction
   */
  public boolean getCurrentInBend() {
    return getElbowPosition() - getShoulderPosition() < 0;
  }

  /**
   * Stop the shoulder and elbow motors.
   */
  public void stopArm() {
    elbowLeft.stopMotor();
    elbowRight.stopMotor();
    shoulderLeft.stopMotor();
    shoulderRight.stopMotor();
  }

  /**
   * Set whether the arm's default state is the stow position.
   *
   * @param Stowing set <code>true</code> if the arm is stowing.
   */
  public void setStowing(boolean Stowing) {
    stowing = Stowing;
  }

  /**
   * Get whether the wrist has reached near its target position.
   *
   * @return <code>true</code> if the wrist is near its target position.
   */
  // public boolean wristFinishedMoving() {
      // TODO: This doesn't work for unrestricted angular domains.
    // return Math.abs(getWristPosition() - wristTarget) < 0.2;
  // }

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
    SignalLogger.writeDouble("shoulder error", shoulderTarget - getShoulderPosition());
    SignalLogger.writeDouble("elbow error", elbowTarget - getElbowPosition());
    SignalLogger.writeDouble("wrist error", wristTarget - getWristPosition());
    SignalLogger.writeDouble("arm pose x", getArmPosition().getX());
    SignalLogger.writeDouble("arm pose y", getArmPosition().getY());
    SignalLogger.writeDouble("elbow pos", getElbowPosition());
    SignalLogger.writeDouble("shoulder pos", getShoulderPosition());
    SignalLogger.writeDouble("wrist flip pos", getWristPosition());
    // SmartDashboard.putNumber("shoulder velocity", getShoulderVelocity());
    // SmartDashboard.putNumber("elbow velocity", getElbowVelocity());
    // SmartDashboard.putNumber("left elbow amp", elbowLeft.getDutyCycle().getValueAsDouble());
    SignalLogger.writeDouble("right shoulder amp", shoulderRight.getStatorCurrent().getValueAsDouble());
    SignalLogger.writeDouble("left shoulder amp", shoulderLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("right shoulder amp", shoulderRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("left shoulder amp", shoulderLeft.getStatorCurrent().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
