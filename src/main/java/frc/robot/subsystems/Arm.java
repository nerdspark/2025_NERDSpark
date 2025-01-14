// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private TalonFX shoulder;
  private TalonFX elbow;
  private TalonFX wrist;
  private TalonFX hand; //TODO find a better name for this

  /** Creates a new Arm. */
  public Arm() {

    shoulder = new TalonFX(0, "canivore1");
    elbow = new TalonFX(0, "canivore1");
    wrist = new TalonFX(0, "canivore1");
    hand = new TalonFX(0, "canivore1");

 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
