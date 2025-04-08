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
import frc.robot.subsystems.ElevIndexer;

/** Add your docs here. */
public class SubsystemActions {
    public static Command deployCoral(ElevIndexer elevIndexer, DoubleSupplier target, DoubleSupplier shootVoltage) {
        return new SequentialCommandGroup(
            elevIndexer.setElevatorPosition(target),
            new WaitUntilCommand(() -> elevIndexer.elevatorAtTarget()),
            new WaitCommand(0.08),
            elevIndexer.shoot(shootVoltage)//, 
            // new WaitCommand(0.2),
            // elevIndexer.home()
        );
    }
    public static Command placeCoral(ElevIndexer elevIndexer, DoubleSupplier target, DoubleSupplier shootVoltage) {
        return new SequentialCommandGroup(
            elevIndexer.setElevatorPosition(target),
            new WaitUntilCommand(() -> elevIndexer.elevatorAtTarget()),
            new WaitCommand(1.0),
            elevIndexer.shoot(shootVoltage)//, 
        //     new WaitCommand(1.0),
        //     elevIndexer.home()
        );
    }

  
}
