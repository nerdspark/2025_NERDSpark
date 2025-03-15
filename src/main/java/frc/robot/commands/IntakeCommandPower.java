// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommandPower extends Command {private Intake intake;
    private Supplier<Double> power, intakePower;
  /** Creates a new IntakeCommand. */
  public IntakeCommandPower(Intake intake, Supplier<Double> power, Supplier<Double> intakePower) {
    this.intake = intake;
        this.power = power;
        this.intakePower = intakePower;
        addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setDeployPower(power.get());
    intake.setGrabberIntake(intakePower.get());
    // if((intake.getRangeIntakeDetected()) && (intake.getRangeIntakeDistance() < 0.1)){
    //   intake.setGrabberIntake(IntakeConstants.intakePassive);
    // }
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
