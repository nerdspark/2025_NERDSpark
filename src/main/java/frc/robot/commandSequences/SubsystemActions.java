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
    public static Command prepareDropOffAlgae(CoralManipulator coralManipulator) {
      return new SequentialCommandGroup(
        coralManipulator.intakeToProcessor(),
        coralManipulator.setIntakeVoltage(CoralConstants.intakeAlgaeVoltage)
      );
    }
    public static Command dropOffAlgae(CoralManipulator coralManipulator) {
      return new SequentialCommandGroup(
        coralManipulator.intakeToProcessor(), 
        new WaitUntilCommand(() -> coralManipulator.deployAtTarget()),
        coralManipulator.setIntakeVoltage(CoralConstants.processorVoltage), 
        new WaitCommand(1.0),
        coralManipulator.intakeToRetract(),
        new WaitCommand(0.5),
        coralManipulator.stopIntake()
      );
    }
    public static Command intakeAlgae(CoralManipulator coralManipulator) {
      return new SequentialCommandGroup(
        coralManipulator.setCoralStateCommand(coralState.algaeInIntake), 
        coralManipulator.intakeToAlgaeDeploy(),
        coralManipulator.setIntakeVoltage(CoralConstants.intakeAlgaeVoltage)//,
        // new WaitUntilCommand(() -> coralManipulator.getIntakeSensor()), 
        // coralManipulator.intakeToAlgaeHome(), 
        // new WaitCommand(0.5),
        // coralManipulator.setIntakeVoltage(CoralConstants.neutralAlgaeVoltage)
      );
    }
    public static Command panicButton(CoralManipulator coralManipulator) {
      return new ParallelCommandGroup(
        coralManipulator.setCoralStateCommand(coralState.empty),
        coralManipulator.intakeToDeploy(), 
        coralManipulator.setIntakeVoltage(-5), 
        coralManipulator.setIndexerVoltage(CoralConstants.indexerTransferVoltage), 
        coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.panic.height), 
        coralManipulator.shoot(CoralConstants.elevatorLevel.panic.shootVoltage)
      );
    }
    public static Command resetDeploy(CoralManipulator coralManipulator) {
      return new SequentialCommandGroup(
        coralManipulator.setDeployVoltage(-1.5), 
        new WaitCommand(0.1),
        new WaitUntilCommand(() -> coralManipulator.deployAmpTriggered()), 
        coralManipulator.resetDeploy(), 
        coralManipulator.stopDeploy());
    }
    public static Command resetElevator(CoralManipulator coralManipulator) {
      return new SequentialCommandGroup(
        coralManipulator.setElevatorLeftVoltage(-0.3), 
        new WaitCommand(0.05),
        // new WaitUntilCommand(() -> coralManipulator.elevatorLeftAmpTriggered()), 
        coralManipulator.resetElevatorLeft(), 
        coralManipulator.stopElevatorLeft()).alongWith(new SequentialCommandGroup(
          coralManipulator.setElevatorRightVoltage(-0.3), 
          new WaitCommand(0.05),
          // new WaitUntilCommand(() -> coralManipulator.elevatorRightAmpTriggered()), 
          coralManipulator.resetElevatorRight(), 
          coralManipulator.stopElevatorRight())).withTimeout(0.1);
    }
    public static Command placeCoral(CoralManipulator coralManipulator, elevatorLevel level) {
        return new SequentialCommandGroup(
            // new WaitUntilCommand(() -> coralManipulator.getCoralState().equals(coralState.coralInElevator)),
            coralManipulator.setElevatorPosition(level.height),
            new WaitUntilCommand(() -> coralManipulator.elevatorAtTarget()),
            new WaitCommand(0.08),
            coralManipulator.shoot(level.shootVoltage), 
            // new WaitCommand(1.0/level.shootVoltage), 
            coralManipulator.setCoralStateCommand(coralState.empty));
    }
    public static Command placeCoralAuto(CoralManipulator coralManipulator, elevatorLevel level) {
      return new SequentialCommandGroup(
          // new WaitUntilCommand(() -> coralManipulator.getCoralState().equals(coralState.coralInElevator)),
          coralManipulator.setElevatorPosition(level.height),
          new WaitUntilCommand(() -> coralManipulator.elevatorAtTarget()),
          // new WaitCommand(0.08),
          coralManipulator.shoot(level.shootVoltage), 
          // new WaitCommand(1.0/level.shootVoltage), 
          coralManipulator.setCoralStateCommand(coralState.empty));
  }

  public static Command transferCoral(CoralManipulator coralManipulator) {
    return new SequentialCommandGroup(
      coralManipulator.intakeToTransfer(),
      new WaitUntilCommand(() -> coralManipulator.deployAtTarget()),
      coralManipulator.setIndexerVoltage(CoralConstants.indexerTransferVoltage),
      coralManipulator.setIntakeVoltage(CoralConstants.intakeTransferVoltage), 
      coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.transfer.height),
      coralManipulator.shoot(CoralConstants.elevatorLevel.transfer.shootVoltage),
      new WaitUntilCommand(() -> !coralManipulator.getIntakeSensor()), 
      new WaitCommand(0.0), 
      coralManipulator.intakeToRetract(),
      new WaitUntilCommand(() -> coralManipulator.getIndexerSensor()), 
      new WaitUntilCommand(() -> !coralManipulator.getIndexerSensor()), 
      coralManipulator.shoot(CoralConstants.shooterRewindVoltage), 
      new WaitCommand(0.02),
      new WaitUntilCommand(() -> coralManipulator.getIndexerSensor()), 
      coralManipulator.stopShooter(), 
      coralManipulator.stopIndexer(),
      coralManipulator.stopIntake(),
      // coralManipulator.elevatorToHome(), 
      coralManipulator.setCoralStateCommand(coralState.coralInIndexer)
      ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);

  }

  public static Command transferCoralForAuto(CoralManipulator coralManipulator) {
    return new SequentialCommandGroup(
      coralManipulator.intakeToTransfer(),
      new WaitUntilCommand(() -> coralManipulator.deployAtTarget()),
      coralManipulator.setIndexerVoltage(CoralConstants.indexerTransferVoltage),
      coralManipulator.setIntakeVoltage(CoralConstants.intakeTransferVoltage), 
      coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.transfer.height),
      coralManipulator.shoot(CoralConstants.autoShootVoltageTransfer),
      new WaitUntilCommand(() -> !coralManipulator.getIntakeSensor()), 
      new WaitCommand(0.0), 
      coralManipulator.intakeToRetract(),
      new WaitUntilCommand(() -> coralManipulator.getIndexerSensor()), 
      new WaitUntilCommand(() -> !coralManipulator.getIndexerSensor()), 
      coralManipulator.stopShooter(), 
      coralManipulator.stopIndexer(),
      coralManipulator.stopIntake(),
      // coralManipulator.elevatorToHome(), 
      coralManipulator.setCoralStateCommand(coralState.coralInIndexer)
      ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);

  }
    
  public static Command transferCoralToIndexer(CoralManipulator coralManipulator) {
    return new SequentialCommandGroup(
      coralManipulator.intakeToTransfer(),
      new WaitUntilCommand(() -> coralManipulator.deployAtTarget()),
      coralManipulator.setIndexerVoltage(CoralConstants.indexerTransferVoltage),
      coralManipulator.setIntakeVoltage(CoralConstants.intakeTransferVoltage), 
      coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.transfer.height));
      // new WaitUntilCommand(() -> coralManipulator.getIndexerSensor()), 
      // coralManipulator.setCoralStateCommand(coralState.coralInIndexer),
      // coralManipulator.stopIndexer(), 
      // coralManipulator.stopIntake(), 
      // coralManipulator.retractIntake());
  }
  
  public static Command transferCoralToElevator(CoralManipulator coralManipulator) {
    return new SequentialCommandGroup(
      coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.transfer.height),
      new WaitUntilCommand(() -> coralManipulator.elevatorAtTarget()),
      coralManipulator.setIndexerVoltage(CoralConstants.indexerTransferVoltage),
      coralManipulator.shoot(CoralConstants.elevatorLevel.transfer.shootVoltage),
      new WaitUntilCommand(() -> !coralManipulator.getIndexerSensor()),
      coralManipulator.setCoralStateCommand(coralState.coralInIndexer)); 
      // coralManipulator.stopIndexer(),
      // coralManipulator.stopShooter(),
      // coralManipulator.elevatorToHome());
  }
  
}
