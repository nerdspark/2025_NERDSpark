// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
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
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
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
  private boolean ampTriggered, ampTriggerStarted = false;
  private double startTime, toAmpTriggerStartTime = 0;
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
      climbConfig.Slot0 = new Slot0Configs()
      .withKP(ClimbConstants.kP)
      .withKI(ClimbConstants.kI)
      .withKD(ClimbConstants.kD);
      climbLeft
          .getConfigurator()
          .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake)));
      climbRight
          .getConfigurator()
          .apply(climbConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake)));
      
      resetPosition();
  }
  
  public void setPower(double power) {
    climbLeft.set(power);
    climbRight.set(power);
  }
  public void setClimb() {
    if (ampTriggered) {
      double current = -ClimbConstants.currentLimit * MathUtil.clamp(Math.abs(Timer.getFPGATimestamp() - startTime), 0, 1);
      climbLeft.setControl(new TorqueCurrentFOC(current)); 
      climbRight.setControl(new TorqueCurrentFOC(current)); 
    } else {
      climbLeft.set(ClimbConstants.power);
      climbRight.set(ClimbConstants.power);
    }
  }
  public void stop() {
    climbLeft.stopMotor(); 
    climbRight.stopMotor();
  }
  public double getCurrent() {
    return (climbLeft.getStatorCurrent().getValueAsDouble() + climbRight.getStatorCurrent().getValueAsDouble())/2;
  }
  public void resetPosition() {
    climbLeft.setPosition(0);
    climbRight.setPosition(0);
  }
  public void setPosition(double position) {
    climbLeft.setControl(new PositionVoltage(position)); 
    climbRight.setControl(new PositionVoltage(position)); 
    
  }
  public double getPosition() {
    return (climbLeft.getPosition().getValueAsDouble() + climbRight.getPosition().getValueAsDouble())/2;
  }

  public void setCurrentLimit(double currentLimit) {
    climbConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(currentLimit)
          .withStatorCurrentLimitEnable(true);
          climbLeft.getConfigurator().apply(climbConfig);
          climbRight.getConfigurator().apply(climbConfig);
        }
  public boolean climbed() {
    return getPosition() < ClimbConstants.climbedPosition;
  }

  public double getCurrentLimit() {
    return climbConfig.CurrentLimits.StatorCurrentLimit;
  }
  public Command climb() {
    return new Command() {
      @Override
      public void initialize() {
        startTime = Timer.getFPGATimestamp();
      }
      @Override
      public void execute() {
        setClimb();
        if (Math.abs(getCurrent()) > ClimbConstants.ampTriggeredCurrentLimit){
          if (!ampTriggerStarted) {
            toAmpTriggerStartTime = Timer.getFPGATimestamp();
          }
          if (!ampTriggered && ampTriggerStarted) {
            if (Math.abs(toAmpTriggerStartTime - Timer.getFPGATimestamp()) < 0.05) {
              ampTriggered = true;
              startTime = Timer.getFPGATimestamp();
            } 
          }
          
        }
      }
      @Override
      public void end(boolean interrupted) {
        stop();
      }
    };
  }
  public Command deploy() {
    return new InstantCommand(() -> setPosition(ClimbConstants.deployPosition));
  }
  public Command stopCommand() {
    return new InstantCommand(() -> stop());
  }




  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("climb pos", getPosition());
    SmartDashboard.putNumber("climb right pos", climbRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("climb left pos", climbLeft.getPosition().getValueAsDouble());
    SignalLogger.writeDouble("climb left amp", climbLeft.getStatorCurrent().getValueAsDouble());
    SignalLogger.writeDouble("climb right amp", climbRight.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("climb left amp", climbLeft.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("climb right amp", climbRight.getStatorCurrent().getValueAsDouble());


  }
}
