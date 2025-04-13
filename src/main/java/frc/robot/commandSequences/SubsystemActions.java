// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandSequences;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.coralState;
import frc.robot.Constants.CoralConstants.elevatorLevel;
import frc.robot.subsystems.CoralManipulator;

/** Add your docs here. */
public class SubsystemActions {
    // public static Command placeCoral(CoralManipulator coralManipulator, double target, double shootVoltage) {
    //     return new SequentialCommandGroup(
    //         coralManipulator.setElevatorPosition(target),
    //         new WaitUntilCommand(() -> coralManipulator.elevatorAtTarget()),
    //         new WaitCommand(0.08),
    //         coralManipulator.shoot(shootVoltage), 
    //         new WaitCommand(1.0), 
    //         coralManipulator.setCoralStateCommand(coralState.empty)//, 
    //     //     new WaitCommand(1.0),
    //     //     elevIndexer.home()
    //     );
    // }
    public static Command resetDeploy(CoralManipulator coralManipulator) {
      return new SequentialCommandGroup(
        coralManipulator.setDeployVoltage(-1), 
        new WaitCommand(0.1),
        new WaitUntilCommand(() -> coralManipulator.deployAmpTriggered()), 
        coralManipulator.resetDeploy(), 
        coralManipulator.stopDeploy());
    }
    public static Command placeCoral(CoralManipulator coralManipulator, elevatorLevel level) {
        return new SequentialCommandGroup(
            // new WaitUntilCommand(() -> coralManipulator.getCoralState().equals(coralState.coralInElevator)),
            coralManipulator.setElevatorPosition(level.height),
            new WaitUntilCommand(() -> coralManipulator.elevatorAtTarget()),
            new WaitCommand(0.08),
            coralManipulator.shoot(level.shootVoltage), 
            new WaitCommand(1.0/level.shootVoltage), 
            coralManipulator.setCoralStateCommand(coralState.empty));
    }
    
  public static Command transferCoralToIndexer(CoralManipulator coralManipulator) {
    return new SequentialCommandGroup(
      coralManipulator.transferIntake(),
      new WaitUntilCommand(() -> coralManipulator.deployAtTarget()),
      coralManipulator.setIndexerVoltage(CoralConstants.indexerTransferVoltage),
      coralManipulator.setIntakeVoltage(CoralConstants.intakeTransferVoltage));
      // new WaitUntilCommand(() -> coralManipulator.getIndexerSensor()), 
      // coralManipulator.setCoralStateCommand(coralState.coralInIndexer),
      // coralManipulator.stopIndexer(), 
      // coralManipulator.stopIntake(), 
      // coralManipulator.retractIntake());
  }
  
  public static Command transferCoralToElevator(CoralManipulator coralManipulator) {
    return new SequentialCommandGroup(
      coralManipulator.setElevatorPosition(CoralConstants.elevatorTransferPosition),
      new WaitUntilCommand(() -> coralManipulator.elevatorAtTarget()),
      coralManipulator.setIndexerVoltage(CoralConstants.indexerTransferVoltage),
      coralManipulator.shoot(CoralConstants.shooterTransferVoltage));
      // new WaitUntilCommand(() -> !coralManipulator.getIndexerSensor()),
      // coralManipulator.setCoralStateCommand(coralState.coralInElevator), 
      // coralManipulator.stopIndexer(),
      // coralManipulator.stopShooter(),
      // coralManipulator.elevatorToHome());
  }
  
}
