// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.reefManagement;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Reef extends SubsystemBase {
  public static L1Section[] L1 = new L1Section[6];
  public static L2 L2 = new L2();

  /** Creates a new Reef. */
  public Reef() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
