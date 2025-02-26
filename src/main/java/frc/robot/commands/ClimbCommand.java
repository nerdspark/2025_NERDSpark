
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {
  private double m_position;
  private Climb m_climber;
  /** Creates a new ClimbCommand. */
  public ClimbCommand(Climb climber, double position) {
    // Use addRequirements() here to declare subsystem dependencies. 
    m_climber=climber;
    m_position=position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_climber.setPosition(m_position);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setServoPosition(m_position);
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
