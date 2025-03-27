// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.reefManagement;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class L2 extends SubsystemBase {
  public boolean[] rows = new boolean[12];
  public boolean[] algae = new boolean[3];
  /** Creates a new L2. */
  public L2() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
