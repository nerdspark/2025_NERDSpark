// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import dev.doglog.DogLog;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.AutoDropoff;
import frc.robot.Constants.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;

public class DriveBetweenCages extends Command {
  private final CommandSwerveDrivetrain drive;
  private Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  double loopPeriodSecs = AutoDropoff.loopPeriodSecs;
  private  final ProfiledPIDController driveController = AutoDropoff.driveController;
  private final ProfiledPIDController thetaController = AutoDropoff.thetaController;
 private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;
  private Supplier<Translation2d> joystickVector;
    private final SwerveRequest.ApplyRobotSpeeds driveToPoseRequest = new SwerveRequest.ApplyRobotSpeeds();

  public DriveBetweenCages(CommandSwerveDrivetrain drive, Supplier<Pose2d> poseSupplier, Supplier<Translation2d> joystickVector) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    this.joystickVector = joystickVector;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  
  @Override
  public void initialize() {

    driveController.setTolerance(Constants.Vision.TRANSLATION_TOLERANCE_X, Constants.Vision.VELOCITY_TOLERANCE_X);
    thetaController.setTolerance(Constants.Vision.ROTATION_TOLERANCE,Constants.Vision.VELOCITY_TOLERANCE_OMEGA);
    // Reset all controllers
    var currentPose = drive.getState().Pose;
    
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(
                drive.getCurrentRobotChassisSpeeds().vxMetersPerSecond, drive.getCurrentRobotChassisSpeeds().vyMetersPerSecond)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(drive.getState().Pose.getTranslation())
                        .getAngle())
                        .unaryMinus()
                .getX()));
    thetaController.reset(currentPose.getRotation().getRadians(), drive.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
    lastSetpointTranslation = drive.getState().Pose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Get current and target pose
    var currentPose = drive.getState().Pose;
    var targetPose = poseSupplier.get();
    SmartDashboard.putString("currentDrivePoseLine", drive.getState().Pose.toString());
    SmartDashboard.putString("taretPoseLine", poseSupplier.get().toString());
    Transform2d error = targetPose.minus(currentPose);
        SignalLogger.writeDouble("LINEX", error.getX());
        SignalLogger.writeDouble("LINEY", error.getY());
        SignalLogger.writeDouble("LINEO", error.getRotation().getDegrees());

        System.out.println("LINEx: " + error.getX() + "; Y: " + error.getY() + "; O: " + error.getRotation().getDegrees());
        
    // Calculate drive speed

    driveErrorAbs = error.getTranslation().getY();


    double driveVelocityScalar =
             driveController.calculate(driveErrorAbs, 0.0);

    // if (driveErrorAbs < driveController.getPositionTolerance()) {driveVelocityScalar = 0.0;}

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(new Translation2d(driveController.getSetpoint().position, 0.0), new Rotation2d()))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
         thetaController.calculate(
                currentPose.getRotation().getRadians(), AllianceFlipUtil.apply(Rotation2d.kZero).getRadians());
    thetaErrorAbs =
        Math.abs((currentPose.getRotation().minus(targetPose.getRotation())).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    SmartDashboard.putNumber("drive error abs", driveErrorAbs);

    Translation2d driveVelocity = new Translation2d(Math.copySign(driveVelocityScalar, driveErrorAbs), Rotation2d.kCCW_90deg);
        // new Pose2d(
        //         new Translation2d(),
        //         poseSupplier.get().getRotation())
        //     .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
        //     .getTranslation();

    Translation2d joystickAddition = new Translation2d(joystickVector.get().getX(), 0);// new Translation2d(joystickVector.get().getNorm() * joystickVector.get().getAngle().minus(joystickDirection).getCos(), joystickDirection);
    driveVelocity = driveVelocity.times(1).plus(joystickAddition).times(AllianceFlipUtil.shouldFlip() ? -1 : 1);




    SmartDashboard.putNumber("velX", driveVelocity.getX());
    SmartDashboard.putNumber("velY", driveVelocity.getY());
    drive.setControl(driveToPoseRequest.withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(),driveVelocity.getY(),
    thetaVelocity, drive.getState().Pose.getRotation())).withDriveRequestType(DriveRequestType.Velocity));

  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());

  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {

    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }

      // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;//this.atGoal();
    }

    
  
    
}