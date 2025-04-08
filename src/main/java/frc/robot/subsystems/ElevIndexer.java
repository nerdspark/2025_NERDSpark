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
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevIndexer extends SubsystemBase {
  private TalonFX elevatorLeft, elevatorRight, shooter, indexer;
  private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  private double targetPosition = 0;

  /** Creates a new ElevIndexer. */
  public ElevIndexer() {
    shooter = new TalonFX(1);
    indexer = new TalonFX(2);
    elevatorLeft = new TalonFX(3);
    elevatorRight = new TalonFX(4);
    shooterConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(ElevatorConstants.shooterCurrentLimit)
      .withStatorCurrentLimitEnable(true);
    shooterConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.05);
    shooter
      .getConfigurator()
      .apply(shooterConfig.withMotorOutput(new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)));
    elevatorConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(ElevatorConstants.elevatorCurrentLimit)
      .withStatorCurrentLimitEnable(true);
      elevatorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.elevatorSensorRatio);
    elevatorConfig.Slot0 = new Slot0Configs()
      .withKP(ElevatorConstants.kP)
      .withKI(ElevatorConstants.kI)
      .withKD(ElevatorConstants.kD)
      .withKG(ElevatorConstants.kG)
      .withKS(ElevatorConstants.kS)
      .withGravityType(GravityTypeValue.Elevator_Static);
    elevatorConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ElevatorConstants.elevatorRampRate);
    elevatorConfig.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(ElevatorConstants.forwardLimit)
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(ElevatorConstants.reverseLimit);
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
  public void setShooter(double voltage) {
    shooter.setControl(new VoltageOut(voltage));
  }
  public void setElevatorControl(ControlRequest request) {
    elevatorLeft.setControl(request);
    elevatorRight.setControl(request);
  }
  public void setElevatorPosition(double position) {
    targetPosition = position;
    setElevatorControl(new PositionVoltage(position).withSlot(0));
  }
  public boolean elevatorAtTarget() {
    return Math.abs((elevatorLeft.getPosition().getValueAsDouble() + elevatorRight.getPosition().getValueAsDouble())/2 - targetPosition) < ElevatorConstants.elevatorTolerance;
  }
  public Command setElevatorPosition(DoubleSupplier position) {
    return new InstantCommand(() -> setElevatorPosition(position.getAsDouble()));
  }
  public Command home() {
    return new SequentialCommandGroup(
      setElevatorPosition(() -> ElevatorConstants.homePos),
      stopShooter(), 
            new WaitCommand(0.5),
            stopElevator());
  }
  public Command shoot() {
    return new InstantCommand(() -> 
      shooter.setControl(new VoltageOut(2))
    );
  }
  public Command shoot(DoubleSupplier voltage) {
    return new InstantCommand(() -> 
      shooter.setControl(new VoltageOut(voltage.getAsDouble()))
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
