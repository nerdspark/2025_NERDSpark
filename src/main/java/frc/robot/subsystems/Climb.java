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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.GripperCommand;

public class Climb extends SubsystemBase {
  private TalonFX climbLeft, climbRight;
  private TalonFXConfiguration climbConfig = new TalonFXConfiguration();
  /** Creates a new Gripper. */
  public Climb() {
    climbLeft = new TalonFX(ClimbConstants.kLeftID, ArmConstants.armCanBus);
    climbRight = new TalonFX(ClimbConstants.kRightID, ArmConstants.armCanBus);
    climbConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ClimbConstants.currentLimit)
          .withStatorCurrentLimitEnable(true);
      climbConfig.Feedback = new FeedbackConfigs()
          .withFeedbackRotorOffset(0)
          .withSensorToMechanismRatio(1);
      climbConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ClimbConstants.rampRate);
      climbLeft
          .getConfigurator()
          .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake)));
      climbRight
          .getConfigurator()
          .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake)));
      
  }
  
  public void setPower(double power) {
    climbLeft.set(power); 
    
  }

  public void setCurrentLimit(double currentLimit) {
    climbConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(currentLimit)
          .withStatorCurrentLimitEnable(true);
    climbLeft.getConfigurator().apply(climbConfig);
  }

  public double getCurrentLimit() {
    return climbConfig.CurrentLimits.StatorCurrentLimit;
  }
  public Command outCommand() {
    return new InstantCommand(() ->setPower(0.05));
  }




  @Override
  public void periodic() {
    
    

  }
}
