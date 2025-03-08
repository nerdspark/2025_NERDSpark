// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;
import frc.robot.Constants.ArmVelocityGains;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX intakeDeployMotor;
  private TalonFX intakeGrabberMotor;
  private CANrange sensorIntake;
  private boolean hasCoral = false;
  public boolean finishedMovingToTransfer = false;
  private double deployTarget = 0;

  /** Creates a new Intake. */
  public Intake() {
    intakeDeployMotor = new TalonFX(IntakeConstants.intakeDeployMotorPort, IntakeConstants.intakeCANBus);
    intakeGrabberMotor = new TalonFX(IntakeConstants.intakeGrabberMotorPort, IntakeConstants.intakeCANBus);

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
        .withSensorToMechanismRatio(IntakeConstants.deployGearRatio);
        intakeGrabberMotorConfig.Feedback = new FeedbackConfigs()
        .withFeedbackRotorOffset(IntakeConstants.grabberOffset)
        .withSensorToMechanismRatio(IntakeConstants.grabberGearRatio);

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

        intakeDeployMotor.setPosition(IntakeConstants.deployOffset);
        sensorIntake = new CANrange(IntakeConstants.intakeRangePort, ArmConstants.armCanBus);
        CANrangeConfiguration sensorIntakeConfig = new CANrangeConfiguration();
        sensorIntake.getConfigurator().apply(sensorIntakeConfig);
  }

  public double getIntakeDeployPosition() {
    double intake = intakeDeployMotor.getPosition().getValueAsDouble();
    // SmartDashboard.putNumber("intake", intake);
    deployTarget = intake;
    return intake;
  }  

  public double getIntakeGrabberPosition() { // returns relative to deploy motor
    return intakeGrabberMotor.getPosition().getValueAsDouble();
  }
  public void stopIntake() {
    intakeDeployMotor.stopMotor();
    intakeGrabberMotor.stopMotor();
  }

  public void resetOffsets() {
    intakeGrabberMotor.setPosition(Constants.ArmConstants.elbowOffset);
    intakeDeployMotor.setPosition(Constants.ArmConstants.elbowOffset);
  }

  public void resetIntakeDeployPosition(double offset) {
    intakeDeployMotor.setPosition(offset);
  }
  public void setDeployTarget(double target){
    intakeDeployMotor.setControl(new PositionVoltage(target).withPosition(target));
  }
  public void setGrabberIntake(double target){
    intakeGrabberMotor.set(target);
  }public void setDeployPower(double target){
    intakeDeployMotor.set(target);
  }
  public double getRangeIntakeDistance() {
    return sensorIntake.getDistance().getValueAsDouble();
  }
  public boolean getRangeIntakeDetected(){
    return sensorIntake.getIsDetected().getValue();
  }
  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void periodic() {
    
    finishedMovingToTransfer =getIntakeDeployPosition() < IntakeConstants.intakeTransferPosition;
    

    // SmartDashboard.putNumber("intake range", getRangeIntakeDistance());
    // SmartDashboard.putBoolean("intake detected", getRangeIntakeDetected());
    // This method will be called once per scheduler run
  }
}
