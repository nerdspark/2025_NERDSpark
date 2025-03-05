
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  public Climb() {
    
  }

// private Servo ClimbServoRight = new Servo(8);
// private Servo ClimbServoLeft = new Servo(9);

public void setServoPosition(double value){
  // ClimbServoRight.setAngle(value + ArmConstants.rightServoOffset);
  // ClimbServoLeft.setAngle(-value - ArmConstants.leftServoOffset);
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // ClimbServoRight.getAngle();
  }


}
