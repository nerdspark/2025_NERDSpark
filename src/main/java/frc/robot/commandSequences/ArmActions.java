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
import frc.robot.Constants.ArmSetpoints;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommandPathToPoint;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

/** Add your docs here. */
public class ArmActions {

  /** grab coral from funnel */
  public static Command grabFromFunnel(Arm arm, Gripper gripper) {
    return new SequentialCommandGroup(
          gripper.coralIntakeCommand(),
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7].withWrist(Math.PI)).withTimeout(0.2), 
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7]).withTimeout(0.3), 
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[0].withWrist(ArmSetpoints.armSetPoints[7].wrist)).withTimeout(0.4), 
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7]).withTimeout(0.1), 
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7].withWrist(Math.PI)).withTimeout(0.1), 
          arm.goToHome().withTimeout(0.2), 
          gripper.neutralCommand());
  }

  /** move arm to ground intake position and begin intaking */
  public static Command groundIntake(Arm arm, Gripper gripper) {
    return new ArmCommandPathToPoint(arm, () -> 8).alongWith(gripper.coralIntakeCommand());
  }

    /** move arm to desired setpoint to drop coral on reef
     * @param setPointIndex level on the reef
     */
  public static Command armToCoralReef(Arm arm, IntSupplier setPointIndex) {
    return new ArmCommand(arm, setPointIndex);
  }

  /** tilt wrist downwards and release coral 
    * @param setPointIndex level on the reef
  */
  public static Command dunkDropCoral(Arm arm, Gripper gripper, IntSupplier setPointIndex) {
    return new ParallelCommandGroup(
      new ArmCommand(arm, () -> ArmSetpoints.armSetPointsDunk[setPointIndex.getAsInt()]), 
      new WaitCommand(0.2).andThen(gripper.spitOutCommand())).withTimeout(0.5);
  }

  /** tilt wrist downwards manually 
    * @param setPointIndex level on the reef
    @param dunkScalar amount to dunk by set by driver
  */
  public static Command dunkCoral(Arm arm, IntSupplier setPointIndex, DoubleSupplier dunkScalar) {
    return new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[setPointIndex.getAsInt()].interpolate(ArmSetpoints.armSetPointsDunk[setPointIndex.getAsInt()], dunkScalar.getAsDouble()));
  }

    /** position arm to remove algae while rolling rollers inwards
        * @param higherLevel true if the algae is at a higher level (L3.5); false if the algae is at a lower level (L2.5)
    */
    public static Command removeAlgae(Arm arm, Gripper gripper, BooleanSupplier higherLevel) {
      return new ArmCommand(arm, () -> higherLevel.getAsBoolean() ? 6 : 5).alongWith(gripper.algaeIntakeCommand());
    }

    /** position arm to drop off algae in barge */
    public static Command armToAlgaeBarge(Arm arm) {
      return new ArmCommand(arm, () -> 9);
    }

    /** spin rollers to drop off algae in barge */
    public static Command shootAlgaeBarge(Gripper gripper) {
      return gripper.spitOutCommand();
    }
}
