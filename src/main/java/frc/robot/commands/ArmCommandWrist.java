// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommandWrist extends Command {
  private Arm arm;
  private Supplier<Double> wristFlip, wristTwist;
  /** Creates a new ArmCommandWrist. */
  public ArmCommandWrist(Arm arm, Supplier<Double> wristFlip, Supplier<Double> wristTwist) {
    this.arm = arm;
    this.wristFlip = wristFlip;
    this.wristTwist = wristTwist;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setWristFlipPosition(wristFlip.get());
    arm.setWristTwistPosition(wristTwist.get());
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
