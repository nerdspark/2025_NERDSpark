// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.coralState;

public class CoralManipulator extends SubsystemBase {
  private CoralConstants.coralState coralState = CoralConstants.coralState.empty;
  private CoralConstants.elevatorLevel elevatorLevel = CoralConstants.elevatorLevel.home;
  private TalonFX elevatorLeft, elevatorRight, shooter, indexer, deploy, intake;
  private CANrange intakeSensor, indexerSensor;
  private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  private double targetPosition = 0;

  /** Creates a new ElevIndexer. */
  public CoralManipulator() {
    shooter = new TalonFX(CoralConstants.shooterID);
    indexer = new TalonFX(CoralConstants.indexerID);
    elevatorLeft = new TalonFX(CoralConstants.elevatorLeftID);
    elevatorRight = new TalonFX(CoralConstants.elevatorRightID);
    deploy = new TalonFX(CoralConstants.deployID);
    intake = new TalonFX(CoralConstants.intakeID);
    intakeSensor = new CANrange(CoralConstants.intakeSensorID);
    indexerSensor = new CANrange(CoralConstants.indexerSensorID);
    shooterConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(CoralConstants.shooterCurrentLimit)
      .withStatorCurrentLimitEnable(true);
    shooterConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.05);
    shooter
      .getConfigurator()
      .apply(shooterConfig.withMotorOutput(new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)));
    elevatorConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(CoralConstants.elevatorCurrentLimit)
      .withStatorCurrentLimitEnable(true);
      elevatorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(CoralConstants.elevatorSensorRatio);
    elevatorConfig.Slot0 = new Slot0Configs()
      .withKP(CoralConstants.kP)
      .withKI(CoralConstants.kI)
      .withKD(CoralConstants.kD)
      .withKG(CoralConstants.kG)
      .withKS(CoralConstants.kS)
      .withGravityType(GravityTypeValue.Elevator_Static);
    elevatorConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(CoralConstants.elevatorRampRate);
    elevatorConfig.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(CoralConstants.forwardLimit)
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(CoralConstants.reverseLimit);
    elevatorLeft
      .getConfigurator()
      .apply(elevatorConfig.withMotorOutput(new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)));
    elevatorRight
      .getConfigurator()
      .apply(elevatorConfig.withMotorOutput(new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)));
    elevatorLeft.setPosition(0);
    elevatorRight.setPosition(0);
  }
  public coralState getCoralState() {
    return coralState;
  }
  
  public boolean getIntakeSensor() {
    return intakeSensor.getDistance().getValueAsDouble() < CoralConstants.intakeSensorTriggerDistance && intakeSensor.getIsDetected().getValue();
  }
  public boolean getIndexerSensor() {
    return indexerSensor.getDistance().getValueAsDouble() < CoralConstants.indexerSensorTriggerDistance && indexerSensor.getIsDetected().getValue();
  }
  public void setCoralState(coralState coralState) {
    this.coralState = coralState;
  }
  public Command setCoralStateCommand(coralState coralState) {
    return new InstantCommand(() -> setCoralState(coralState));
  }
  public Command setIndexerVoltage(double voltage){
    return new InstantCommand(() -> indexer.setControl(new VoltageOut(voltage)));
  }
  public Command setIntakeVoltage(double voltage){
    return new InstantCommand(() -> intake.setControl(new VoltageOut(voltage)));
  }
  public Command stopIntake() {
    return new InstantCommand(() -> intake.stopMotor());
  }
  public Command stopIndexer() {
    return new InstantCommand(() -> indexer.stopMotor());
  }
  public Command deployIntake() {
    return new InstantCommand(() -> deploy.setControl(new PositionVoltage(CoralConstants.deployPositionIntake)));
  }
  public Command retractIntake() {
    return new InstantCommand(() -> deploy.setControl(new PositionVoltage(CoralConstants.homePositionIntake)));
  }


  public void setShooter(double voltage) {
    shooter.setControl(new VoltageOut(voltage));
  }
  public void setElevatorControl(ControlRequest request) {
    elevatorLeft.setControl(request);
    elevatorRight.setControl(request);
  }
  public void setElevPosition(double position) {
    targetPosition = position;
    setElevatorControl(new PositionVoltage(position).withSlot(0));
  }
  public boolean elevatorAtTarget() {
    return Math.abs((elevatorLeft.getPosition().getValueAsDouble() + elevatorRight.getPosition().getValueAsDouble())/2 - targetPosition) < CoralConstants.elevatorTolerance;
  }
  public Command setElevatorPosition(double position) {
    return new InstantCommand(() -> setElevPosition(position));
  }
  public Command elevatorHome() {
    return new SequentialCommandGroup(
      setElevatorPosition(CoralConstants.homePos),
      stopShooter(), 
            new WaitCommand(0.5),
            stopElevator());
  }
  
  public Command shoot(double voltage) {
    return new InstantCommand(() -> 
      shooter.setControl(new VoltageOut(voltage))
    );
  }
  public Command stopShooter() {
    return new InstantCommand(() -> shooter.stopMotor()); 
  }
  public Command stopElevator() {
    return new InstantCommand(() -> elevatorLeft.stopMotor()).alongWith(new InstantCommand(() -> elevatorRight.stopMotor())); 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elev left pos", elevatorLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("elev right pos", elevatorRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("elevator left amp", elevatorLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("elevator right amp", elevatorRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("shooter pos", shooter.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("shooter amp", shooter.getStatorCurrent().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
