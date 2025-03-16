// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommandGripperGroundPickup extends Command {
  private Gripper gripper;
  private double timeToAct = Timer.getFPGATimestamp();
  /** Creates a new ArmCommandGripperAutoClose. */
  public ArmCommandGripperGroundPickup(Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    addRequirements(gripper);
  }
  // public static Command armCommandGroundPickup(Arm arm, Gripper gripper) {
  //   return new SequentialCommandGroup(
  //     new ArmCommandGripperGroundPickup(gripper).deadlineFor(new ArmCommandPathToPoint(arm, () -> 14))
  //     );
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.openGripperStrong();
    timeToAct = Timer.getFPGATimestamp();
    System.out.println("start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double minRange = Math.min(gripper.getRangeLeftDistance(), Math.min(gripper.getRangeMiddleDistance(), gripper.getRangeRightDistance()));
    // boolean rangeTrue = (minRange < max);

    
    if (!gripper.getDetected()){
      timeToAct = Timer.getFPGATimestamp();
    }
    System.out.println("gripper detect " + gripper.getDetected());
    System.out.println("timetoact - current " + Math.abs(timeToAct - Timer.getFPGATimestamp()));

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.closeGripper();
    System.out.println("finish");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(timeToAct - Timer.getFPGATimestamp()) > 0.07 && gripper.getDetected();
  }
}
