// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.util.ArmPoint;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import javax.crypto.SecretKeyFactorySpi;

import org.opencv.core.Point;

public class ArmCommand extends Command {
    private Arm arm;
    private Supplier<ArmPoint> point;
    private Supplier<Boolean> inBend;
    private ArmPoint prevPoint = new ArmPoint(new Translation2d());
    /** Creates a new ArmCommand. */
    public ArmCommand(Arm arm, Supplier<ArmPoint> point) {
        this.arm = arm;
        this.point = point;
        inBend = () -> point.get().inBend;

        addRequirements(arm);
    }
    public ArmCommand(Arm arm, IntSupplier index) {
        this.arm = arm;
        this.point = () -> ArmSetpoints.armSetPoints[index.getAsInt()];
        inBend = () -> this.point.get().inBend;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // arm.resetEncoders();
        System.out.println("armcommand init " + point.get().position.toString() + "wrist " + point.get().wrist);
        setPosition();
        
    }
    public void setPosition() {
        System.out.println("armcommand refresh - xy:" + point.get().position.toString() + " - wrist:" + point.get().wrist);
        arm.setArmPosition(point.get().position, inBend.get());
        arm.setWristTarget(point.get().wrist);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // arm.getArmPosition();
        if ((!point.get().position.equals(prevPoint.position)) || !(point.get().wrist == prevPoint.wrist)) {
            setPosition();
            prevPoint = point.get();
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // arm.setElbowPosition(-ArmConstants.elbowOffset);
        // arm.setShoulderPosition(-ArmConstants.shoulderOffset);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
      }
}
