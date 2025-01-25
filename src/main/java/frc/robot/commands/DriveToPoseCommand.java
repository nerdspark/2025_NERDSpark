// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Vision.*;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Inventory;
import frc.robot.util.PIDToPosition;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

/** An example command that uses an example subsystem. */
public class DriveToPoseCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CommandSwerveDrivetrain drivetrainSubsystem;
    PIDToPosition PID = new PIDToPosition();
    private final Supplier<Pose2d> targetPoseSupplier;

    private final Supplier<Pose2d> currentPoseProvider;
    private final Supplier<Rotation2d> robotAngle;
    // private  final Pose2d goalPose;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
            new TrapezoidProfile.Constraints(Constants.Vision.MAX_VELOCITY, Constants.Vision.MAX_ACCELARATION);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
            new TrapezoidProfile.Constraints(Constants.Vision.MAX_VELOCITY, Constants.Vision.MAX_ACCELARATION);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(
            Constants.Vision.MAX_VELOCITY_ROTATION, Constants.Vision.MAX_ACCELARATION_ROTATION);
    private final ProfiledPIDController xController = new ProfiledPIDController(
            Constants.Vision.kPXController, Constants.Vision.kIXController, Constants.Vision.kIXController, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(
            Constants.Vision.kPYController, Constants.Vision.kIYController, Constants.Vision.kDYController, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(
            Constants.Vision.kPThetaController,
            Constants.Vision.kIThetaController,
            Constants.Vision.kDThetaController,
            OMEGA_CONSTRATINTS);

    private final SwerveRequest.ApplyRobotSpeeds driveToPoseRequest = new SwerveRequest.ApplyRobotSpeeds();
        //Supplier<Translation2d> joystick;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveToPoseCommand(
            CommandSwerveDrivetrain drivetrainSubsystem,
            Supplier<Pose2d> poseProvider,
            Supplier<Pose2d> goalPoseSupplier,
            Supplier<Rotation2d> robotAngle) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.currentPoseProvider = poseProvider;
        this.targetPoseSupplier = goalPoseSupplier;
        this.robotAngle = robotAngle;

        xController.setTolerance(Constants.Vision.TRANSLATION_TOLERANCE_X, Constants.Vision.VELOCITY_TOLERANCE_X);
        yController.setTolerance(Constants.Vision.TRANSLATION_TOLERANCE_Y,Constants.Vision.VELOCITY_TOLERANCE_Y);
        omegaController.setTolerance(Constants.Vision.ROTATION_TOLERANCE,Constants.Vision.VELOCITY_TOLERANCE_OMEGA);
        omegaController.enableContinuousInput(-180.0, 180.0);
        omegaController.setIZone(Constants.Vision.IZone);
        xController.setIZone(Constants.Vision.kIzoneX);
        yController.setIZone(Constants.Vision.kIzoneY);
        
        addRequirements(drivetrainSubsystem);
    }

    /** Drives to the specified pose when passed a target pose */
    public DriveToPoseCommand(
            CommandSwerveDrivetrain drivetrainSubsystem,
            Supplier<Pose2d> poseProvider,
            Pose2d pose,
            Supplier<Rotation2d> robotAngle,
            Inventory inventory) {
        this(drivetrainSubsystem, poseProvider, () -> pose, robotAngle);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SmartDashboard.putString("DriveToPoseCommand", "Initialize");

        var robotPose = currentPoseProvider.get();
        omegaController.reset(
                robotAngle.get().getDegrees(),
                drivetrainSubsystem.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond * 180.0 / Math.PI);
        xController.reset(robotPose.getX(), drivetrainSubsystem.getCurrentRobotChassisSpeeds().vxMetersPerSecond);
        yController.reset(robotPose.getY(), drivetrainSubsystem.getCurrentRobotChassisSpeeds().vyMetersPerSecond);

        SmartDashboard.putNumber(
                "YawVelocity",
                drivetrainSubsystem.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond * 180.0 / Math.PI);
        SmartDashboard.putNumber(
                "FieldVelocityX", drivetrainSubsystem.getCurrentRobotChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber(
                "FieldVelocityY", drivetrainSubsystem.getCurrentRobotChassisSpeeds().vyMetersPerSecond);

        omegaController.setGoal(targetPoseSupplier.get().getRotation().getDegrees());
        xController.setGoal(targetPoseSupplier.get().getX());
        yController.setGoal(targetPoseSupplier.get().getY());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // SmartDashboard.putString("DriveToPoseCommand", "Execute");
        
        var robotPose = currentPoseProvider.get();
        SmartDashboard.putNumber("DriveToPoseCommand robotPose.X", robotPose.getX());
        SmartDashboard.putNumber("DriveToPoseCommand robotPose.Y", robotPose.getY());
        SmartDashboard.putNumber(
                "DriveToPoseCommand robotAngle", robotAngle.get().getDegrees());

        SmartDashboard.putNumber(
                "DriveToPoseCommand goalPose.X", targetPoseSupplier.get().getX());
        SmartDashboard.putNumber(
                "DriveToPoseCommand goalPose.Y", targetPoseSupplier.get().getY());
        SmartDashboard.putNumber(
                "DriveToPoseCommand goalPose.Angle",
                targetPoseSupplier.get().getRotation().getDegrees());
        
        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }
    
        var omegaSpeed = omegaController.calculate(robotAngle.get().getDegrees());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        SmartDashboard.putNumber("DriveToPose X Speed", xSpeed);
        SmartDashboard.putNumber("DriveToPose Y Speed", ySpeed);

        SmartDashboard.putNumber("DriveToPose omega Speed", omegaSpeed);

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                omegaSpeed,
                robotAngle.get());
        drivetrainSubsystem.setControl(driveToPoseRequest.withSpeeds(chassisSpeeds));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // ChassisSpeeds zeroChassisSpeeds = new ChassisSpeeds();
        // drivetrainSubsystem.setControl(driveToPoseRequest.withSpeeds(zeroChassisSpeeds));
        drivetrainSubsystem.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
}
