// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommandGripperPosition extends Command {
  private Gripper gripper;
  private Supplier<Double> position;
  public ArmCommandGripperPosition(Gripper gripper, Supplier<Double> position) {
    this.gripper = gripper;
    this.position = position;
    addRequirements(gripper);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.setCurrentLimitStrong();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      gripper.setGripperPosition(position.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // gripper.stopGripper();
    // gripper.setCurrentLimitWeak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
