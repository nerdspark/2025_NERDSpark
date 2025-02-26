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
public class ArmCommandGripper extends Command {
  private Gripper gripper;
  private Supplier<Boolean> gripperClose;
  private double startTime = Timer.getFPGATimestamp();
  /** Creates a new ArmCommandGripperOpen. */
  public ArmCommandGripper(Gripper gripper, Supplier<Boolean> gripperClose) {
    this.gripper = gripper;
    this.gripperClose = gripperClose;
    addRequirements(gripper);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    if(gripperClose.get()){
      gripper.closeGripper();
    } else {
      gripper.openGripper();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Timer.getFPGATimestamp() - startTime > 1) && !gripperClose.get()){
      return true;
    }
    return false;
  }
}
