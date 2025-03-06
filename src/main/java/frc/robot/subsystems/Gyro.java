// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.generated.TunerConstants;

public class Gyro extends SubsystemBase {
  private final Pigeon2 gyro = new Pigeon2(TunerConstants.kPigeonId);
  
  public static final double gyroP = 2;
  public static final double gyroI = 0.0;
  public static final double gyroD = 0.00;

  private PIDController GyroPID = new PIDController(gyroP, gyroI, gyroD);

  public double targetAngle = 0;

  private Rotation2d gyroResetAngle = new Rotation2d();

  public Gyro() {
    GyroPID.enableContinuousInput(0.0, 2*Math.PI);
  }

  public Rotation2d getGyro() {
    return new Rotation2d(-gyro.getYaw().getValueAsDouble()*Math.PI/180).minus(gyroResetAngle);
  }

  public void resetGyro() {
    gyroResetAngle = getGyro().plus(gyroResetAngle);
    targetAngle = 0;
  }
}