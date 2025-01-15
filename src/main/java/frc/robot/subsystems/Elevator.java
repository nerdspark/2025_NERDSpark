// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;// Constants.setHeights


public class Elevator extends SubsystemBase {
  private SparkMax elevatorMotor;
  private PIDController elevatorPID;
  private RelativeEncoder elevatorEncoder;
  private double targetPosition;
  private ElevatorFeedforward feedforward;
  /** Creates a new Elevator. */
  public Elevator() {
    SparkMax elevatorMotor = new SparkMax(Constants.ElevatorID, null); 
    PIDController elevatorPID = new PIDController(Constants.kPElevator, Constants.kIElevator, Constants.kDElevator);
    ElevatorFeedforward feedforward = new ElevatorFeedforward(Constants.kSElevator, Constants.kGElevator, Constants.kVElevator, Constants.kAElevator);
    RelativeEncoder elevatorEncoder = elevatorMotor.getAlternateEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      elevatorMotor.set(elevatorPID.calculate(elevatorEncoder.getPosition(), targetPosition) + 
      feedforward.calculate(targetPosition));
  }
}
