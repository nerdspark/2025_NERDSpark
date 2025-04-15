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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private TalonFX winch;
  private TalonFXConfiguration winchConfig = new TalonFXConfiguration();
  // private double startTime, toAmpTriggerStartTime = 0;
  /** Creates a new Gripper. */
  public Climb() {
    winch = new TalonFX(ClimbConstants.winchId, ClimbConstants.canBus);
    winchConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ClimbConstants.currentLimit)
          .withStatorCurrentLimitEnable(true);
      winchConfig.Feedback = new FeedbackConfigs()
          .withFeedbackRotorOffset(0)
          .withSensorToMechanismRatio(1);
      winchConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ClimbConstants.rampRate);
      winchConfig.Slot0 = new Slot0Configs()
      .withKP(ClimbConstants.kP)
      .withKI(ClimbConstants.kI)
      .withKD(ClimbConstants.kD);
      winch
          .getConfigurator()
          .apply(winchConfig.withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake)));
      
      
      resetPosition();
  }
  public Command extend() {
    return new InstantCommand(() -> setPosition(ClimbConstants.deployPosition));
  }
  public Command contract() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setPower(ClimbConstants.power)), 
      new WaitUntilCommand(() -> getPosition() > ClimbConstants.climbedPosition),
      stopCommand());
  }
  public Command returnToHome() {
    return new InstantCommand(() -> setPosition(ClimbConstants.homePosition));
  }
  public void setPower(double power) {
    winch.set(power);
  }
  // public void setClimb() {
  //   if (ampTriggered) {
  //     double current = ClimbConstants.currentLimit * MathUtil.clamp(Math.abs(Timer.getFPGATimestamp() - startTime), 0, 1);
  //     climbLeft.setControl(new TorqueCurrentFOC(current)); 
  //     climbRight.setControl(new TorqueCurrentFOC(current)); 
  //   } else {
  //     climbLeft.set(ClimbConstants.power);
  //     climbRight.set(ClimbConstants.power);
  //   }
  // }
  public void stop() {
    winch.stopMotor(); 
  }
  public double getCurrent() {
    return (winch.getStatorCurrent().getValueAsDouble() );
  }
  public void resetPosition() {
    winch.setPosition(ClimbConstants.offset);
  }
  public void setPosition(double position) {
    winch.setControl(new PositionVoltage(position)); 
    
  }
  public double getPosition() {
    return (winch.getPosition().getValueAsDouble());
  }

  public void setCurrentLimit(double currentLimit) {
    winchConfig.CurrentLimits = new CurrentLimitsConfigs()
          .withStatorCurrentLimit(currentLimit)
          .withStatorCurrentLimitEnable(true);
          winch.getConfigurator().apply(winchConfig);
        }
  public boolean climbed() {
    return false;//Math.abs(getPosition()) < Math.abs(ClimbConstants.climbedPosition);
  }

  public double getCurrentLimit() {
    return winchConfig.CurrentLimits.StatorCurrentLimit;
  }
  public Command climb() {
    return new InstantCommand(() -> setPower(ClimbConstants.power));//setFOC(ClimbConstants.currentLimit));
  }
  public void setFOC(double current){
    winch.setControl(new TorqueCurrentFOC(current)); 
  }
  // public Command climbSwitchToFOC() {
  //   return new Command() {
  //     @Override
  //     public void initialize() {
  //       startTime = Timer.getFPGATimestamp();
  //     }
  //     @Override
  //     public void execute() {
  //       setClimb();
  //       if (Math.abs(getCurrent()) > ClimbConstants.ampTriggeredCurrentLimit){
  //         if (!ampTriggerStarted) {
  //           toAmpTriggerStartTime = Timer.getFPGATimestamp();
  //         }
  //         if (!ampTriggered && ampTriggerStarted) {
  //           if (Math.abs(toAmpTriggerStartTime - Timer.getFPGATimestamp()) < 0.05) {
  //             ampTriggered = true;
  //             startTime = Timer.getFPGATimestamp();
  //           } 
  //         }
          
  //       }
  //     }
  //     @Override
  //     public void end(boolean interrupted) {
  //       stop();
  //     }
  //   };
  // }
  public Command deploy() {
    return new InstantCommand(() -> setPosition(ClimbConstants.deployPosition));
  }
  public Command stopCommand() {
    return new InstantCommand(() -> stop());
  }




  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("winch pos", getPosition());
    SmartDashboard.putNumber("winch amp", winch.getStatorCurrent().getValueAsDouble());


  }
}
