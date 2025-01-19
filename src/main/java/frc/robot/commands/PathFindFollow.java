// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TeleopAutomationConstants;

/** A command that runs pathfindThenFollowPath based on the current drive mode. */
public class PathFindFollow extends Command {
  private Command scoreCommand;
  private Command pathRun;

  @Override
  public void initialize() {

    scoreCommand = getReefAutonPathCommand();
    scoreCommand.schedule();
  }

  @Override
  public void execute() {
      scoreCommand.cancel();
      scoreCommand = getReefAutonPathCommand();
      scoreCommand.schedule();
    }


  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    scoreCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return pathRun.isFinished();
  }

  /** Runs a new autonomous path based on the current drive mode. */
  public Command getReefAutonPathCommand() {
    PathPlannerPath ampPath = null;
    try {
        ampPath = PathPlannerPath.fromPathFile("BlueTeleopHighGoal");
    } catch (FileVersionException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
    } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
    } catch (ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
    }
    PathConstraints constraints =
        new PathConstraints(
            TeleopAutomationConstants.PATH_VELOCITY_MAX,
            TeleopAutomationConstants.PATH_ACCELERATION_MAX,
            TeleopAutomationConstants.PATH_ANGULAR_VELOCITY_MAX,
            TeleopAutomationConstants.PATH_ANGULAR_ACCELERATION_MAX);
    if (ampPath != null) {
        pathRun = AutoBuilder.pathfindThenFollowPath(ampPath, constraints);
    } else {
        // Handle the case where ampPath is null
        pathRun = Commands.none();
    }
    return Commands.sequence(pathRun);
  }

}