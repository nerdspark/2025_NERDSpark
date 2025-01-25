// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Inventory extends SubsystemBase {
  /** Creates a new Inventory. */
  private boolean A = false;
  private boolean B = false;
  private boolean X = false;
  private boolean Y = false;
  public Inventory() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("inventory", A);

  }

  public boolean getAstatus() {
    return A;
    
  }


  public void setAstatus(boolean status) {
    A = status;
  }
}
