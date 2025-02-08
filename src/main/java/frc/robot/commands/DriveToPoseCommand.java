// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.Vision.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;

/** An example command that uses an example subsystem. */
public class DriveToPoseCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CommandSwerveDrivetrain drivetrainSubsystem;

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
            Supplier<Rotation2d> robotAngle) {
        this(drivetrainSubsystem, poseProvider, () -> pose, robotAngle);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // DogLog.log("DriveToPoseCommand", "Initialize");

        var robotPose = currentPoseProvider.get();
        omegaController.reset(
                robotAngle.get().getDegrees(),
                drivetrainSubsystem.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond * 180.0 / Math.PI);
        xController.reset(robotPose.getX(), drivetrainSubsystem.getCurrentRobotChassisSpeeds().vxMetersPerSecond);
        yController.reset(robotPose.getY(), drivetrainSubsystem.getCurrentRobotChassisSpeeds().vyMetersPerSecond);

        // DogLog.log(
        //         "DriveToPoseCommand/YawVelocity",
        //         drivetrainSubsystem.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond * 180.0 / Math.PI);
        // DogLog.log(
        //         "DriveToPoseCommand/FieldVelocityX", drivetrainSubsystem.getCurrentRobotChassisSpeeds().vxMetersPerSecond);
        // DogLog.log(
        //         "DriveToPoseCommand/FieldVelocityY", drivetrainSubsystem.getCurrentRobotChassisSpeeds().vyMetersPerSecond);

        omegaController.setGoal(targetPoseSupplier.get().getRotation().getDegrees());
        xController.setGoal(targetPoseSupplier.get().getX());
        yController.setGoal(targetPoseSupplier.get().getY());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // SmartDashboard.putString("DriveToPoseCommand", "Execute");

        var robotPose = currentPoseProvider.get();
        // DogLog.log("DriveToPoseCommand/robotPose.X", robotPose.getX());
        // DogLog.log("DriveToPoseCommand/robotPose.Y", robotPose.getY());
        // DogLog.log("DriveToPoseCommand/robotAngle", robotAngle.get().getDegrees());

        // DogLog.log("DriveToPoseCommand/goalPose.X", targetPoseSupplier.get().getX());
        // DogLog.log("DriveToPoseCommand/goalPose.Y", targetPoseSupplier.get().getY());
        // DogLog.log("DriveToPoseCommand/goalPose.Angle", targetPoseSupplier.get().getRotation().getDegrees());

        // DogLog.log("DriveToPoseCommand/Error.x", targetPoseSupplier.get().getX() - robotPose.getX());
        // DogLog.log("DriveToPoseCommand/Error.y", targetPoseSupplier.get().getY() - robotPose.getY());
        // DogLog.log("DriveToPoseCommand/Error.theta", targetPoseSupplier.get().getRotation().getDegrees() - robotAngle.get().getDegrees());


        if((targetPoseSupplier.get().getTranslation().getDistance(currentPoseProvider.get().getTranslation()) 
        > AllianceFlipUtil.apply(FieldConstants.Reef.center).getDistance(currentPoseProvider.get().getTranslation()))){
        //    DogLog.log("DriveToPoseCommand/GoalPose", "Out of Range");
        }else{
        //     DogLog.log("DriveToPoseCommand/GoalPose", "In Range");
        }       


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

        // DogLog.log("DriveToPoseCommand/X Speed", xSpeed);
        // DogLog.log("DriveToPoseCommand/Y Speed", ySpeed);
        // DogLog.log("DriveToPoseCommand/Omega Speed", omegaSpeed);

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
        return ((targetPoseSupplier.get().getTranslation().getDistance(currentPoseProvider.get().getTranslation()) 
        > AllianceFlipUtil.apply(FieldConstants.Reef.center).getDistance(currentPoseProvider.get().getTranslation())) ||
        xController.atGoal() && yController.atGoal() && omegaController.atGoal());
    }
    
}
