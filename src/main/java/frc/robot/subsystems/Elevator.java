// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Point;

import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;// Constants.setHeights


public class Elevator extends SubsystemBase {
  private SparkMax elevMotor;
  private SparkMax wristMotor;
  private PIDController elevPID;
  private SparkAbsoluteEncoder elevEncoder;
  private SparkAbsoluteEncoder wristEncoder;
  private double targetPosition;
  private ElevatorFeedforward elevFF;
  public Mechanism2d m_elev;
  private MechanismRoot2d m_elev_root;
  private MechanismLigament2d m_elev_structure;
  private MechanismLigament2d m_wrist;
  private Point example1;
  // MechanismRoot2d elevMechRoot = new MechanismRoot2d("",0,0);
  /** Creates a new Elevator. */
  public Elevator() {
     elevMotor = new SparkMax(Constants.elevatorID, MotorType.kBrushless); 
     elevPID = new PIDController(Constants.kPElevator, Constants.kIElevator, Constants.kDElevator);
     elevFF = new ElevatorFeedforward(Constants.kSElevator, Constants.kGElevator, Constants.kVElevator, Constants.kAElevator);
     elevEncoder = elevMotor.getAbsoluteEncoder(); 
     wristMotor = new SparkMax(Constants.wristID, MotorType.kBrushless); 
    wristEncoder = wristMotor.getAbsoluteEncoder();
    
    m_elev = new Mechanism2d(Constants.elevWidth, Constants.elevHeight);
    m_elev_root = m_elev.getRoot("elevator", Constants.elevXPos, Constants.elevYPos); //
    m_elev_structure = m_elev_root.append(new MechanismLigament2d("elevator", Constants.kElevatorMinLength, 90, 6, new Color8Bit(Color.kGreen)));
    m_wrist = m_elev_structure.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
  }

  public void setTargetPosition(double target) {
    targetPosition = target;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevMotor.set(elevPID.calculate(elevEncoder.getPosition(), targetPosition) + elevFF.calculate(targetPosition));
    // DogLog.
    m_elev_structure.setLength(Constants.kElevatorMinLength + elevEncoder.getPosition());
    m_wrist.setAngle(wristEncoder.getPosition());
  }

  public void initElevDashboard() {
    // TODO Auto-generated method stub
    SmartDashboard.putData("elevator mechanism", m_elev);
  }
}
