// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

/** Add your docs here. */
public class ArmActions {

  /** grab coral from funnel */
  public static Command grabFromFunnel(Arm arm, Gripper gripper) {
    return new ParallelRaceGroup(
      new RollerCommand(gripper, () -> 1.0, () -> 20),
        new SequentialCommandGroup(
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[10].withWristFlip(Math.PI)).withTimeout(0.3), 
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[10]).withTimeout(0.3), 
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[10].add(new Translation2d(0, -8))).withTimeout(0.2), 
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[10]).withTimeout(0.3), 
          new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[10].withWristFlip(Math.PI)).withTimeout(0.3)));
  }

    /** move arm to desired setpoint to drop coral on reef
     * @param setPointIndex level on the reef
     */
  public static Command moveToCoralReef(Arm arm, IntSupplier setPointIndex) {
    return new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[setPointIndex.getAsInt()]);
  }

  /** tilt wrist downwards and release coral 
    * @param setPointIndex level on the reef
  */
  public static Command dunkDropCoral(Arm arm, Gripper gripper) {
    return new ParallelCommandGroup(
      new WristCommand(arm, () -> arm.wristFlipTarget + -1), 
      new WaitCommand(0.2).andThen(new RollerCommand(gripper, () -> -1, () -> 20))).withTimeout(0.5);
  }

    /** position arm to remove algae while rolling rollers inwards
        * @param higherLevel true if the algae is at a higher level (L3.5); false if the algae is at a lower level (L2.5)
    */
    public static Command removeAlgae(Arm arm, Gripper gripper, boolean higherLevel) {
      return new ArmCommand(arm, () -> higherLevel ? 6 : 5).alongWith(new RollerCommand(gripper, () -> 1.0, () -> 20));
    }

    /** position arm to drop off algae in barge */
    public static Command moveToAlgaeBarge(Arm arm) {
      return new ArmCommand(arm, () -> 12);
    }

    /** spin rollers to drop off algae in barge */
    public static Command shootAlgaeBarge(Gripper gripper) {
      return new RollerCommand(gripper, () -> -1.0, () -> 20);
    }
}
