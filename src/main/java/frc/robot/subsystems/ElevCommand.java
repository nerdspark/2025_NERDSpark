// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevCommand extends Command {
  private final Elevator elev;
  private final CommandXboxController driveController;
  private DoubleSupplier leftX, rightY;

  /** Creates a new ElevCommand. */
  public ElevCommand(Elevator elevSub, DoubleSupplier leftX, DoubleSupplier rightY) {
    // Use addRequirements() here to declare subsystem dependencies.
    elev =  elevSub;
    driveController = new CommandXboxController(Constants.driveControllerPort);
    addRequirements(elevSub);
    this.leftX = leftX;
    this.rightY = rightY;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elev.setElevHeight(leftX.getAsDouble());
    elev.setWristAngle(rightY.getAsDouble());
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
