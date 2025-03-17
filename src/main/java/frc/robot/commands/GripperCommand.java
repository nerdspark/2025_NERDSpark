// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperCommand extends Command {
  /** Creates a new GripperComand. */
  private Gripper gripper;
  private DoubleSupplier power;
  private IntSupplier currentLimit;

  public GripperCommand(Gripper gripper, DoubleSupplier power, IntSupplier currentLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    this.power = power;
    this.currentLimit = currentLimit;

    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripper.setGripperPower(power.getAsDouble());

    gripper.setCurrentLimit(currentLimit.getAsInt());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
