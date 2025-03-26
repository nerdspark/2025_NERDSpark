// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb_Command extends Command {
  private Climb m_climber;
  private Servo servo;



  /** Creates a new Climb_Command. */
  public Climb_Command() {
    }
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setFOC(Constants.ClimbConstants.currentLimit);
    m_climber.setServoPosition(Constants.ClimbConstants.servoOpenPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
    m_climber.setServoPosition(Constants.ClimbConstants.servoOpenPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_climber.getServoPosition()==Constants.ClimbConstants.servoClosePosition){
      return true;
    }
    return false;
  }

}
