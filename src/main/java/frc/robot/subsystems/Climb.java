
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.*;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  public Climb() {
    
  }

private PWM climbServoRight = new PWM(Constants.ClimbConstants.climbServoRightChannel);
private PWM climbServoLeft = new PWM(Constants.ClimbConstants.climbServoLeftChannel);

// public void setServoPosition(double value){
//   // climbServoRight.setAngle(value + ArmConstants.rightServoOffset);
//   // climbServoLeft.setAngle(-value - ArmConstants.leftServoOffset);
// }

public void setServoOpen() {
  climbServoRight.setPosition(Constants.ClimbConstants.setServoOpenRight);
  climbServoLeft.setPosition(Constants.ClimbConstants.setServoOpenLeft);
}
public void setServoClose() {
  climbServoRight.setPosition(Constants.ClimbConstants.closeServoOpenRight);
  climbServoLeft.setPosition(Constants.ClimbConstants.closeServoOpenLeft);
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // ClimbServoRight.getAngle();
  }


}
