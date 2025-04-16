// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
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

import edu.wpi.first.wpilibj.Timer;
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
  private TalonFXConfiguration deployConfig = new TalonFXConfiguration();
  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
  private double targetPositionElevator, targetPositionDeploy = 0;
  private CANrangeConfiguration intakeSensorConfig = new CANrangeConfiguration();
  private CANrangeConfiguration indexerSensorConfig = new CANrangeConfiguration();

  /** Creates a new ElevIndexer. */
  public CoralManipulator() {
    shooter = new TalonFX(CoralConstants.shooterID, CoralConstants.canBus);
    indexer = new TalonFX(CoralConstants.indexerID, CoralConstants.canBus);
    elevatorLeft = new TalonFX(CoralConstants.elevatorLeftID, CoralConstants.canBus);
    elevatorRight = new TalonFX(CoralConstants.elevatorRightID, CoralConstants.canBus);
    deploy = new TalonFX(CoralConstants.deployID, CoralConstants.canBus);
    intake = new TalonFX(CoralConstants.intakeID, CoralConstants.canBus);
    intakeSensor = new CANrange(CoralConstants.intakeSensorID, CoralConstants.canBus);
    indexerSensor = new CANrange(CoralConstants.indexerSensorID, CoralConstants.canBus);
    intakeSensorConfig = new CANrangeConfiguration()
    .withFovParams(new FovParamsConfigs().withFOVRangeX(27).withFOVRangeY(27))
    .withProximityParams(new ProximityParamsConfigs().withProximityThreshold(CoralConstants.intakeSensorTriggerDistance).withMinSignalStrengthForValidMeasurement(2500));
    intakeSensor.getConfigurator().apply(intakeSensorConfig);
    indexerSensorConfig = new CANrangeConfiguration()
    .withFovParams(new FovParamsConfigs().withFOVRangeX(7).withFOVRangeY(7))
    .withProximityParams(new ProximityParamsConfigs().withProximityThreshold(CoralConstants.indexerSensorTriggerDistance).withMinSignalStrengthForValidMeasurement(5000));
    indexerSensor.getConfigurator().apply(indexerSensorConfig);
    configureDeploy();
    configureIntake();
    configureShooter();
    configureElevator();
    configureIndexer(); 
    resetMotors();   
  }
  private void configureIndexer() {
    indexerConfig.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
    indexerConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(CoralConstants.indexerCurrentLimit);
    indexer.getConfigurator().apply(indexerConfig);
  }
  private void configureIntake() {
    intakeConfig.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(CoralConstants.intakeCurrentLimit).withStatorCurrentLimitEnable(true);
    intakeConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.05);
    intakeConfig.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    intake.getConfigurator().apply(intakeConfig);
  }
  private void configureShooter() {
    shooterConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(CoralConstants.shooterCurrentLimit)
      .withStatorCurrentLimitEnable(true);
    shooterConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.05);
    shooter
      .getConfigurator()
      .apply(shooterConfig.withMotorOutput(new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)));
  }
  private void configureDeploy() {
    deployConfig.CurrentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(CoralConstants.deployCurrentLimit)
      .withStatorCurrentLimitEnable(true);
    deployConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(CoralConstants.deploySensorRatio);
    deployConfig.Slot0 = new Slot0Configs()
      .withKP(CoralConstants.kPDeploy)
      .withKI(CoralConstants.kIDeploy)
      .withKD(CoralConstants.kDDeploy)
      .withKG(CoralConstants.kGDeploy)
      .withGravityType(GravityTypeValue.Arm_Cosine);
    deployConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(CoralConstants.deployRampRate);
    deployConfig.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
      .withForwardSoftLimitEnable(false)
      .withForwardSoftLimitThreshold(CoralConstants.forwardLimitDeploy)
      .withReverseSoftLimitEnable(false)
      .withReverseSoftLimitThreshold(CoralConstants.reverseLimitDeploy);
    deployConfig.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast);
    deploy.getConfigurator().apply(deployConfig);
  }
  private void configureElevator() {
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
        .withNeutralMode(NeutralModeValue.Brake)));
  elevatorRight
    .getConfigurator()
    .apply(elevatorConfig.withMotorOutput(new MotorOutputConfigs()
    .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake)));
  }
  
  private void resetMotors() {
    elevatorLeft.setPosition(0);
    elevatorRight.setPosition(0);
    deploy.setPosition(CoralConstants.deployOffset);
  }
  public Command resetDeploy() {
    return new InstantCommand(() -> deploy.setPosition(CoralConstants.deployOffset));
  }
  public Command resetElevatorLeft() {
    return new InstantCommand(() -> elevatorLeft.setPosition(0));
  }
  public Command resetElevatorRight() {
    return new InstantCommand(() -> elevatorRight.setPosition(0));
  }
  public boolean deployAmpTriggered() {
    return Math.abs(deploy.getStatorCurrent().getValueAsDouble()) > 55;
  }
  public coralState getCoralState() {
    return coralState;
  }
  public Command stopDeploy() {
    return new InstantCommand(() -> deploy.stopMotor());
  }
  public Command stopElevatorLeft() {
    return new InstantCommand(() -> elevatorLeft.stopMotor());
  }
  public Command stopElevatorRight() {
    return new InstantCommand(() -> elevatorRight.stopMotor());
  }
  public Command setDeployVoltage(double voltage) {
    return new InstantCommand(() -> deploy.setControl(new VoltageOut(voltage)));
  }
  public void setDeployVoltageFunction(double voltage) {
    deploy.setControl(new VoltageOut(voltage));
  }
  public boolean getIntakeSensor() {
    return intakeSensor.getIsDetected().getValue();
  }
  public boolean getIndexerSensor() {
    return indexerSensor.getIsDetected().getValue();
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
  private void setDeployPosition(double position) {
    deploy.setControl(new PositionVoltage(position));
    targetPositionDeploy = position;
  }
  public Command intakeToDeploy() {
    return new InstantCommand(() -> setDeployPosition(CoralConstants.deployPositionIntake));
  }
  public Command intakeToTransfer() {
    return new InstantCommand(() -> setDeployPosition(CoralConstants.transferPositionIntake));
  }
  public Command intakeToRetract() {
    return new InstantCommand(() ->setDeployPosition(CoralConstants.homePositionIntake));
  }
  public Command intakeToAlgaeDeploy() {
    return new InstantCommand(() ->setDeployPosition(CoralConstants.algaeDeployPositionIntake));
  }
  public Command intakeToAlgaeHome() {
    return new InstantCommand(() ->setDeployPosition(CoralConstants.algaeHomePositionIntake));
  }
  public Command intakeToProcessor() {
    return new InstantCommand(() ->setDeployPosition(CoralConstants.processorPositionIntake));
  }
  


  public void setShooter(double voltage) {
    shooter.setControl(new VoltageOut(voltage));
  }
  public void setElevatorControl(ControlRequest request) {
    elevatorLeft.setControl(request);
    elevatorRight.setControl(request);
  }
  public Command setElevatorLeftVoltage(double voltage) {
    return new InstantCommand(() -> elevatorLeft.setVoltage(voltage));
  }
  public Command setElevatorRightVoltage(double voltage) {
    return new InstantCommand(() -> elevatorRight.setVoltage(voltage));
  }
  
  public boolean elevatorLeftAmpTriggered() {
    return Math.abs(elevatorLeft.getStatorCurrent().getValueAsDouble()) > 10;
  }
  public boolean elevatorRightAmpTriggered() {
    return Math.abs(elevatorRight.getStatorCurrent().getValueAsDouble()) > 10;
  }
  public void setElevPosition(double position) {
    targetPositionElevator = position;
    setElevatorControl(new PositionVoltage(position).withSlot(0));
  }
  public boolean elevatorAtTarget() {
    return Math.abs((elevatorLeft.getPosition().getValueAsDouble() + elevatorRight.getPosition().getValueAsDouble())/2 - targetPositionElevator) < CoralConstants.elevatorTolerance;
  }
  public boolean deployAtTarget() {
    return Math.abs(deploy.getPosition().getValueAsDouble() - targetPositionDeploy) < CoralConstants.deployTolerance;
  }
  public Command setElevatorPosition(double position) {
    return new InstantCommand(() -> setElevPosition(position));
  }
  public Command elevatorToHome() {
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
  public Command intakeCommand() {
    return intakeToDeploy().alongWith(setIntakeVoltage(CoralConstants.intakingVoltage));
  }
  public Command intakeToHome() {
    return intakeToRetract().alongWith(stopIntake());
  }

  @Override
  public void periodic() {
    if (!coralState.equals(coralState.algaeInIntake)) {
      if (getIndexerSensor()) {
        setCoralState(coralState.coralInIndexer);
      } else if (getIntakeSensor()) {
        setCoralState(coralState.coralInIntake);
      }
    }
    SmartDashboard.putNumber("deploy amp", deploy.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("elev left pos", elevatorLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("elev right pos", elevatorRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("elevator left amp", elevatorLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("elevator right amp", elevatorRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("shooter pos", shooter.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("shooter amp", shooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("deploy pose", deploy.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("deploy error", targetPositionDeploy - deploy.getPosition().getValueAsDouble());
    SmartDashboard.putString("coralState", getCoralState().name());
    SmartDashboard.putBoolean("intake detected", getIntakeSensor());
    SmartDashboard.putBoolean("indexer detected", getIndexerSensor());
    SmartDashboard.putNumber("deploy amp", deploy.getStatorCurrent().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
