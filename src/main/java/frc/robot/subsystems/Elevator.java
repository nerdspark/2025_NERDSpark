// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax motor;          
    private final RelativeEncoder encoder;
    private final ProfiledPIDController pid;        
    private final ElevatorFeedforward elevatorFeedForward;
    
    // You'll need to adjust this based on your elevator's gearing
    // private final double COUNTS_PER_INCH = 42.0; // Example value - measure this!
    
    public Elevator() { // Replace SparkMax CANID
        motor = new SparkMax(0, MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        // pid values need tuning, change maxVelocity and maxAcceleration of trapezoid profile
        pid = new ProfiledPIDController(
            Constants.ElevatorConstants.kElevatorKp,
            Constants.ElevatorConstants.kElevatorKi,
            Constants.ElevatorConstants.kElevatorKd,
            new TrapezoidProfile.Constraints(1, 2.45)
        );
        elevatorFeedForward = new ElevatorFeedforward(
            Constants.ElevatorConstants.kElevatorkS,
            Constants.ElevatorConstants.kElevatorkG,
            Constants.ElevatorConstants.kElevatorkV,
            Constants.ElevatorConstants.kElevatorkA
        ); 
    }

    // Returns elevator height in meters
    public double getPositionMeters() {
        return encoder.getPosition() * (2 * Math.PI * Constants.ElevatorConstants.kElevatorDrumRadius) / Constants.ElevatorConstants.kElevatorGearing;
    }

    public double getVelocityMetersPerSecond() {
        return (encoder.getVelocity() / 60) * (2 * Math.PI * Constants.ElevatorConstants.kElevatorDrumRadius)
                / Constants.ElevatorConstants.kElevatorGearing;
    }

    public void reachGoal(double goal) {
        // Change low and high values of MathUtil.clamp
        double voltsOutput = MathUtil.clamp(
                elevatorFeedForward.calculateWithVelocities(getVelocityMetersPerSecond(), pid.getSetpoint().velocity) + pid.calculate(getPositionMeters(), goal),
                -7,     /* low */
            7     /* high */  );
        motor.setVoltage(voltsOutput);
    }

     public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    public boolean aroundHeight(double height){
        return findIfAroundHeight(height, Constants.ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean findIfAroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position (Rotations)", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Velocity (Rotations per Second)", encoder.getVelocity());
        SmartDashboard.putNumber("Elevator Position (Meters)", getPositionMeters());
        SmartDashboard.putNumber("Elevator Velocity (Meters per Second)", getVelocityMetersPerSecond());
        SmartDashboard.putNumber("Elevator Applied Voltage", motor.getAppliedOutput() * motor.getBusVoltage());
        SmartDashboard.putNumber("Elevator Applied Output", motor.getAppliedOutput());
    }
}


