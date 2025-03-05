// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommandPickup extends Command {private Intake intake;
  private Supplier<Double> position, grabberSetPower;
  /** Creates a new IntakeCommandPickup. */
  public IntakeCommandPickup(Intake intake, Supplier<Double> position, Supplier<Double> grabberSetPower) {
    this.intake = intake;
        this.position = position;
        addRequirements(intake);
        this.grabberSetPower = grabberSetPower;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setDeployPosition(position.get());
    if((intake.getRangeIntakeDetected()) && (intake.getRangeIntakeDistance() < 0.1)){
      intake.setGrabberIntake(IntakeConstants.intakePassive);
    } else {
      intake.setGrabberIntake(grabberSetPower.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (intake.getRangeIntakeDetected()) && (intake.getRangeIntakeDistance() < 0.1);
  }
}
