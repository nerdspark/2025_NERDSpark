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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;// Constants.setHeights


public class Elevator extends SubsystemBase {
  private SparkMax elevMotor;
  private PIDController elevPID;
  private RelativeEncoder elevEncoder;
  private double targetPosition;
  private ElevatorFeedforward elevFF;
  private Mechanism2d elevMech;
  private MechanismRoot2d elevMechRoot;
  // MechanismRoot2d elevMechRoot = new MechanismRoot2d("",0,0);
  /** Creates a new Elevator. */
  public Elevator() {
     elevMotor = new SparkMax(Constants.elevatorID, null); 
     elevPID = new PIDController(Constants.kPElevator, Constants.kIElevator, Constants.kDElevator);
     elevFF = new ElevatorFeedforward(Constants.kSElevator, Constants.kGElevator, Constants.kVElevator, Constants.kAElevator);
     elevEncoder = elevMotor.getAlternateEncoder(); 
     elevMech = new Mechanism2d(1, 5);
     elevMechRoot = elevMech.getRoot("elevator", 10.0, 10.0); //
  }

  public void setTargetPosition(double target) {
    targetPosition = target;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      elevMotor.set(elevPID.calculate(elevEncoder.getPosition(), targetPosition) + 
      elevFF.calculate(targetPosition));
  }
}
