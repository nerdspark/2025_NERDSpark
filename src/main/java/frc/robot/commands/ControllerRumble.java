// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ControllerRumble extends Command {
  private Supplier<Boolean> hasCoral;
  private final CommandXboxController joystick = new CommandXboxController(0);


  /** Creates a new ControllerRumble. */
  public ControllerRumble(Supplier<Boolean> hasCoral) {
    this.hasCoral = hasCoral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (hasCoral.get()) {
    //   joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
    // } else {
    //   joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
    // }
    joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
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
