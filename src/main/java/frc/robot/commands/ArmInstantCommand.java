// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.Arm;
import frc.robot.util.ArmPoint;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import javax.crypto.SecretKeyFactorySpi;

import org.opencv.core.Point;

public class ArmInstantCommand extends InstantCommand {
    private Arm arm;
    private Supplier<ArmPoint> point;
    private Supplier<Boolean> inBend;
    private ArmPoint prevPoint = new ArmPoint(new Translation2d());
    /** Creates a new ArmCommand. */
    public ArmInstantCommand(Arm arm, Supplier<ArmPoint> point) {
        this.arm = arm;
        this.point = point;
        inBend = () -> point.get().inBend;

        addRequirements(arm);
    }
    public ArmInstantCommand(Arm arm, IntSupplier index) {
        this.arm = arm;
        this.point = () -> ArmSetpoints.armSetPoints[index.getAsInt()];
        inBend = () -> this.point.get().inBend;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        setPosition();        
    }
    public void setPosition() {
        System.out.println("arm instant command init");
        System.out.println("xy:" + point.get().position.toString());
        System.out.println("wrist:" + point.get().wrist);
        arm.setArmPosition(point.get().position, inBend.get());
        arm.setWristTarget(point.get().wrist);
    }

   
}
