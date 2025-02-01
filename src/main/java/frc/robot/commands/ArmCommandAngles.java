// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import java.util.function.Supplier;

public class ArmCommandAngles extends Command {
    private Arm arm;
    private Supplier<Double> elbow, shoulder;
    /** Creates a new ArmCommand. */
    public ArmCommandAngles(Arm arm, Supplier<Double> elbow, Supplier<Double> shoulder) {
        this.arm = arm;
        this.shoulder = shoulder;
        this.elbow = elbow;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // arm.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // arm.getArmPosition();
        arm.setElbowPosition(elbow.get());
        arm.setShoulderPosition(shoulder.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setElbowPosition(ArmConstants.elbowOffset);
        arm.setShoulderPosition(ArmConstants.shoulderOffset);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
