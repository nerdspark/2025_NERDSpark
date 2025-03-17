// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

/** Add your docs here. */
public class ArmActions {

    /** move arm to desired setpoint
     * @param setPointIndex level on the reef
     */
  public static Command getDropReefOffCommand(Arm arm, Gripper gripper, IntSupplier setPointIndex) {
    return new InstantCommand();
  }

  /** tilt wrist downwards and release coral 
    * @param setPointIndex level on the reef
  */
  public static Command getDunkDropCommand(Arm arm, Gripper gripper, IntSupplier setPointIndex) {
    return new InstantCommand();
  }

    /** position arm to remove algae and roll rollers inwards
        * @param higherLevel true if the algae is at a higher level (L3.5); false if the algae is at a lower level (L2.5)
    */
  public static Command removeAlgaeCommand(Arm arm, Gripper gripper, boolean higherLevel) {
    return new InstantCommand();
  }
  /** grab coral from funnel */
  public static Command funnelIntake(Arm arm, Gripper gripper) {
    return new InstantCommand();
  }

  
}
