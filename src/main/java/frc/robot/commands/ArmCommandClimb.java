// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmGains;
import frc.robot.subsystems.Arm;
import frc.robot.util.ArmPoint;

import java.util.function.Supplier;

public class ArmCommandClimb extends Command {
    private Arm arm;
    private double shoulderPower;
    private double elbowPosition;
    /** Creates a new ArmCommand. */
    public ArmCommandClimb(Arm arm, double shoulderPower, double elbowPosition) {
        this.arm = arm;
this.shoulderPower = shoulderPower;
this.elbowPosition = elbowPosition;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setShoulderAmpLimit(ArmConstants.currentLimitShoulderClimb);
        // arm.setBrakeMode(true);
        // arm.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // arm.getArmPosition();
        arm.setElbowPosition(elbowPosition);
        // if (arm.getShoulderPosition() > ArmConstants.shoulderPositionClimb * 2.0 * Math.PI) {
            arm.setShoulderPower(shoulderPower);
        // } else {
        //     arm.setShoulderPower(0);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setShoulderAmpLimit(ArmConstants.currentLimitShoulder);

        // arm.setElbowPosition(-ArmConstants.elbowOffset);
        // arm.setShoulderPosition(-ArmConstants.shoulderOffset);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;//arm.getShoulderPosition() < ArmConstants.shoulderPositionClimb * 2.0 * Math.PI;
    }
}
