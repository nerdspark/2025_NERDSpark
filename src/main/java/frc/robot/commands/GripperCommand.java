// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperCommand extends InstantCommand {
  /** Creates a new GripperComand. */
  private Gripper gripper;
  private DoubleSupplier power;
  private DoubleSupplier currentLimit;

  public GripperCommand(Gripper gripper, DoubleSupplier power, DoubleSupplier currentLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    this.power = power;
    this.currentLimit = currentLimit;

    addRequirements(gripper);
  }
  public GripperCommand(Gripper gripper, DoubleSupplier power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    this.power = power;
    this.currentLimit = () -> ArmConstants.currentLimitGripper;

    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    gripper.setGripperPower(power.getAsDouble());

    if (currentLimit.getAsDouble() != gripper.getCurrentLimit()) {
      gripper.setCurrentLimit(currentLimit.getAsDouble());
    }
  }

}
