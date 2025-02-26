// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;
import frc.robot.Constants.ArmVelocityGains;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX intakeDeployMotor;
  private TalonFX intakeGrabberMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakeDeployMotor = new TalonFX(IntakeConstants.intakeDeployMotorPort, "rio");
    intakeGrabberMotor = new TalonFX(IntakeConstants.intakeGrabberMotorPort, "rio");

    TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration intakeGrabberMotorConfig = new TalonFXConfiguration();
    intakeDeployMotorConfig.CurrentLimits =  new CurrentLimitsConfigs()
        .withStatorCurrentLimit(IntakeConstants.intakeDeployCurrentLimit)
        .withStatorCurrentLimitEnable(true);
    intakeGrabberMotorConfig.CurrentLimits =  new CurrentLimitsConfigs()
        .withStatorCurrentLimit(IntakeConstants.intakeGrabberCurrentLimit)
        .withStatorCurrentLimitEnable(true);

      intakeDeployMotorConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(IntakeConstants.deployOffset)
        .withSensorToMechanismRatio(IntakeConstants.deployRadPerRot);
        intakeGrabberMotorConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(IntakeConstants.grabberOffset)
        .withSensorToMechanismRatio(IntakeConstants.grabberRadPerRot);

        intakeDeployMotorConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.03);
        intakeGrabberMotorConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.03);

        intakeDeployMotorConfig.Slot0 = new Slot0Configs()
        .withKP(IntakeConstants.deploykP)
        .withKI(IntakeConstants.deploykI)
        .withKD(IntakeConstants.deploykD)
        .withKG(IntakeConstants.deploykG)
        .withGravityType(GravityTypeValue.Arm_Cosine);

        intakeGrabberMotorConfig.Slot0 = new Slot0Configs()
        .withKP(IntakeConstants.grabberkP)
        .withKI(IntakeConstants.grabberkI)
        .withKD(IntakeConstants.grabberkD)
        .withKG(IntakeConstants.grabberkG)
        .withGravityType(GravityTypeValue.Arm_Cosine);

      intakeDeployMotor.getConfigurator()
      .apply(intakeDeployMotorConfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive) // set
        .withNeutralMode(NeutralModeValue.Coast)));

        intakeGrabberMotor.getConfigurator()
      .apply(intakeGrabberMotorConfig.withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive) // set
        .withNeutralMode(NeutralModeValue.Coast)));
  }

  public double getIntakeDeployPosition() {
    return intakeDeployMotor.getPosition().getValueAsDouble();
  }  

  public double getIntakeGrabberPosition() { // returns relative to deploy motor
    return intakeGrabberMotor.getPosition().getValueAsDouble();
  }
  public void stopIntake() {
    intakeDeployMotor.stopMotor();
    intakeGrabberMotor. stopMotor();
  }

  public void resetOffsets() {
    intakeGrabberMotor.setPosition(Constants.ArmConstants.elbowOffset);
    intakeDeployMotor.setPosition(Constants.ArmConstants.elbowOffset);
  }

  public void setIntakeDeployPosition(double target) {
    intakeDeployMotor.setPosition(target);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
