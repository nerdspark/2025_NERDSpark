// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.reefManagement;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class L1Section extends SubsystemBase {
  public boolean[] bottom = new boolean[6];
  public boolean[] top = new boolean[5];
  /** Creates a new L1Section. */
  public L1Section() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
