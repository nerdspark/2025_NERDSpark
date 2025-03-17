// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommandGripperAutoClosePIDOpen extends Command {
  private Gripper gripper;
  private double timeToAct = Timer.getTimestamp();
  private boolean prevRangeDetected, rangeDetected = false;
  private boolean needsAction = true;
  private DoubleSupplier openPosition;
  /** Creates a new ArmCommandGripperAutoClose. */
  public ArmCommandGripperAutoClosePIDOpen(Gripper gripper, DoubleSupplier openPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    this.openPosition = openPosition;
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    needsAction = true;
    prevRangeDetected = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double minRange = Math.min(gripper.getRangeLeftDistance(), Math.min(gripper.getRangeMiddleDistance(), gripper.getRangeRightDistance()));
    // boolean rangeTrue = (minRange < max);
    rangeDetected = gripper.getShelfDetected();

    if(rangeDetected != prevRangeDetected){
      timeToAct = Timer.getTimestamp();
      needsAction = true;
    }

    if (needsAction) {
      if (Math.abs(timeToAct - Timer.getTimestamp()) > 0.02) {
        if (rangeDetected) {
          gripper.closeGripper();
        } else {
          gripper.setGripperPosition(openPosition.getAsDouble());
        }
        needsAction = false;
      }
    }
    prevRangeDetected = rangeDetected;

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
