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
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;
import frc.robot.commands.GripperCommand;

public class Gripper extends SubsystemBase {
  private TalonFX gripper;
  private TalonFXConfiguration gripperConfig = new TalonFXConfiguration();
  /** Creates a new Gripper. */
  public Gripper() {
    gripper = new TalonFX(ArmConstants.gripperMotorPort, ArmConstants.armCanBus);
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ArmConstants.gripperCurrentLimitDefault)
          .withStatorCurrentLimitEnable(true);
        gripperConfig.Feedback = new FeedbackConfigs()
          .withFeedbackRotorOffset(ArmConstants.gripperOffset)
          .withSensorToMechanismRatio(ArmConstants.gripperRadPerRot);
        gripperConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.01);
        gripperConfig.Slot0 = new Slot0Configs()
          .withKP(ArmGains.gripperP)
          .withKI(ArmGains.gripperI)
          .withKD(ArmGains.gripperD);
        gripperConfig.MotorOutput = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        gripper
          .getConfigurator()
          .apply(gripperConfig);
  }
  
  public void setGripperPower(double power) {
    gripper.set(power); 
  }
  public double getGripperVoltage() {
    return gripper.getMotorVoltage().getValueAsDouble();
  }

  public void setCurrentLimit(double currentLimit) {
    gripperConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(currentLimit)
          .withStatorCurrentLimitEnable(true);
    gripper.getConfigurator().apply(gripperConfig);
  }

  public double getCurrentLimit() {
    return gripperConfig.CurrentLimits.StatorCurrentLimit;
  }
  
  public Command coralDefaultCommand() {
    return new GripperCommand(this, 0.3, 10);
  }
  public Command coralIntakeCommand() {
    return new GripperCommand(this, 1.0, 20);
  }
  public Command algaeIntakeCommand() {
    return new GripperCommand(this, 0.6, 30);
  }
  public Command algaeDefaultCommand() {
    return new GripperCommand(this, 0.5, 15);
  }
  public Command spitOutCommand() {
    return new GripperCommand(this, -1.0, 20);
  }
  public Command algaeSpitCommand() {
    return new GripperCommand(this, -1.0, 40);
  }
  public Command neutralCommand() {
    return new GripperCommand(this, 0.0, 20);
  }



  // public boolean getAlgaeDetected() {
  //   for (double distance : prevDistances) {
  //     if (distance > ArmConstants.algaeDistance) {
  //       return false;
  //     }
  //   }
  //   return false;
  // }

  @Override
  public void periodic() {
    
    
    SmartDashboard.putNumber("Gripper velocity", gripper.getVelocity().getValueAsDouble()); // angular velocity (rotations per second)

  }
}
