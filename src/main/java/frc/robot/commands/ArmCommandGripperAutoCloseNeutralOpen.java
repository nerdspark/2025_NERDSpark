// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommandGripperAutoCloseNeutralOpen extends Command {
  private Gripper gripper;
  private double timeToAct = DriverStation.getMatchTime();
  private double min = 0.02;
  private double max = 0.15;
  private boolean prevRangeDetected, rangeDetected = false;
  private boolean close = true;
  /** Creates a new ArmCommandGripperAutoClose. */
  public ArmCommandGripperAutoCloseNeutralOpen(Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    addRequirements(gripper);
  }
  // Used for Neutral open
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double minRange = Math.min(gripper.getRangeLeftDistance(), Math.min(gripper.getRangeMiddleDistance(), gripper.getRangeRightDistance()));
    // boolean rangeTrue = (minRange < max);
    prevRangeDetected = rangeDetected;
    rangeDetected = (gripper.getLeftDetected() && (gripper.getRangeLeftDistance() < max)) || (gripper.getMiddleDetected() && (gripper.getRangeMiddleDistance() < max)) || (gripper.getRightDetected() && (gripper.getRangeRightDistance() < max));
    if(rangeDetected != prevRangeDetected){
      timeToAct = DriverStation.getMatchTime();
    }
    if (Math.abs(timeToAct - DriverStation.getMatchTime()) > 0.02) {
      if (rangeDetected) {
        gripper.closeGripper();
      } else {
        gripper.openGripper();
      }
    }

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