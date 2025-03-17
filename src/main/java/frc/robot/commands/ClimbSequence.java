// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;

/** Add your docs here. */
public class ClimbSequence {
    /** 1 */
    // public static Command prepareToClimb(Arm arm, Intake intake, Climb climb) {
    //     return new ParallelCommandGroup(new ArmCommandAngles(arm, () -> ArmConstants.elbowPositionClimb, () -> Units.degreesToRadians(25)),
    //     new ArmCommandWrist(arm, () -> -0.4, () -> 0.0),
    //     new ClimbCommand(climb, () -> false),
    //     new WaitCommand(0.5).andThen(new IntakeCommand(intake, () -> IntakeConstants.climb, () -> 0.0)));
    // }
    /** 2 */
    // public static Command closeHooks(Arm arm, Intake intake, Climb climb) {
    //     return new ParallelCommandGroup(new ArmCommandAngles(arm, () -> ArmConstants.elbowPositionClimb, () -> Units.degreesToRadians(25)),
    //     new ArmCommandWrist(arm, () -> -0.4, () -> 0.0),
    //     new ClimbCommand(climb, () -> true),
    //     new WaitCommand(0.5).andThen(new IntakeCommand(intake, () -> IntakeConstants.climb, () -> 0.0)));
    // }
    /** 3 */
    // public static Command climbDown(Arm arm, Intake intake, Climb climb){
    //     return new ParallelCommandGroup(
    //         new ArmCommandClimb(arm, ArmConstants.shoulderPowerClimb, ArmConstants.elbowPositionClimb), 
    //         new ArmCommandWrist(arm, () -> -0.4, () -> 0.0),
    //         new IntakeCommand(intake, () -> IntakeConstants.deploy, () -> 0.0), 
    //         new InstantCommand(() -> climb.stopServos()));
    // }
    // /** 4 */
    // public static Command latch(Arm arm, Intake intake, Climb climb){
    //     return new ParallelCommandGroup(
    //         new ArmCommandClimb(arm, ArmConstants.shoulderPowerClimb, ArmConstants.elbowPositionClimb).withTimeout(2), 
    //         new IntakeCommand(intake, () -> IntakeConstants.climbLatch, () -> 0.0));
    // }
}

