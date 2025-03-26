// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDCommand extends Command {
  private Trigger gripperHasGamePiece;
  private Trigger driveTrainFinishedMoving;
  private Supplier<Boolean> bucketHasCoral;
  private LEDSubsytem ledSubsytem;

  /** Creates a new LEDCommand. */
  public LEDCommand(LEDSubsytem ledSubsytem, Trigger gripperHasGamePiece, Supplier<Boolean> bucketHasCoral, Trigger driveTrainFinishedMoving) {
    this.ledSubsytem = ledSubsytem;
    this.gripperHasGamePiece = gripperHasGamePiece;
    this.driveTrainFinishedMoving = driveTrainFinishedMoving;
    this.bucketHasCoral = bucketHasCoral;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Color[] returnColors = ledSubsytem.updateStepColor(gripperHasGamePiece, driveTrainFinishedMoving, bucketHasCoral);
    // ledSubsytem.runPattern(() -> ledSubsytem.getPattern(driveTrainFinishedMoving, bucketHasCoral, gripperHasGamePiece));
      // LEDPattern.steps(
      //   Map.of(
      //     0,
      //     returnColors[0], 
      //     1 / Constants.LEDConstants.numOfSteps, 
      //     returnColors[1], 
      //     2 / Constants.LEDConstants.numOfSteps, 
      //     returnColors[2]
      //   )
      // )
      // .scrollAtRelativeSpeed(Percent.per(Second).of(Constants.LEDConstants.scrollSpeed))
    // );
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
