// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }


  public static Command getAutoDriveCommandReef(
     CommandSwerveDrivetrain drive,
     Supplier<Pose2d> robotPoseSupplier,
     Supplier<Pose2d> goalPoseSupplier,
     Supplier<ReefLevel> reefLevelSupplier,
     DoubleSupplier linearFF_X,
     DoubleSupplier linearFF_Y,
     DoubleSupplier omegaFF) {

       return new DriveToPose(drive, 
       () -> getDriveTargetReef(()->drive.getState().Pose, ()->goalPoseSupplier.get(), ()->reefLevelSupplier.get()),
       () -> getLinearVelocityFromJoysticks(linearFF_X.getAsDouble(),linearFF_Y.getAsDouble()), omegaFF);
          
          
  }

  
  public static Command getAutoDriveCommandStation(
     CommandSwerveDrivetrain drive,
     Supplier<Pose2d> robotPoseSupplier,
     Supplier<Pose2d> goalPoseSupplier,
     DoubleSupplier linearFF_X,
     DoubleSupplier linearFF_Y,
      DoubleSupplier omegaFF) {

       return new DriveToPose(drive, 
       () -> getDriveTargetStation(()->drive.getState().Pose, ()->goalPoseSupplier.get()),
       () -> getLinearVelocityFromJoysticks(linearFF_X.getAsDouble(),linearFF_Y.getAsDouble()), omegaFF);
          
          
  }

  
  public static Command getAutoDriveCommandXY(
     CommandSwerveDrivetrain drive,
     Supplier<Pose2d> robotPoseSupplier,
     Supplier<Pose2d> goalPoseSupplier,
     Supplier<ReefLevel> reefLevelSupplier) {

          return new DriveToPoseCommand(drive, robotPoseSupplier, 
                () -> getDriveTargetReef(()->drive.getState().Pose, ()->goalPoseSupplier.get(), ()->reefLevelSupplier.get()),
                () -> drive.getState().Pose.getRotation());
          
  }

    /** Get drive target. */
  private static Pose2d getDriveTargetReef(Supplier<Pose2d> robotPose, Supplier<Pose2d> goalPose, Supplier<ReefLevel> reefLevel) {
    double maxDistanceReefLineup = 1.5;
    var robot = robotPose.get();
    var goal = goalPose.get();
    // Final line up
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());

    double rotationDiff = offset.getRotation().getDegrees();

    // if(reefLevel.get() == ReefLevel.L5) return goal;
      
    DogLog.log("AutoScoreCommand/offset" , offset);   

    DogLog.log("AutoScoreCommand/goalPose" , goal);
    DogLog.log("AutoScoreCommand/robotPose" , robot);
    DogLog.log("AutoScoreCommand/xDistance" , xDistance);
    DogLog.log("AutoScoreCommand/yDistance" , yDistance);
    DogLog.log("AutoScoreCommand/offsetX" , offset.getX());
    DogLog.log("AutoScoreCommand/offsetY" , offset.getY());
    DogLog.log("AutoScoreCommand/rotationDiff" , rotationDiff);


    double shiftXT = reefLevel.get() == ReefLevel.L5 ? 
      MathUtil.clamp(((yDistance) / (Reef.faceLength)) - ((xDistance + 0.3) / (Reef.faceLength * 3)),
      0.0,1.0) :
      MathUtil.clamp((yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
      0.0,1.0); ;

      double shiftYT = reefLevel.get() == ReefLevel.L5 ? 
      MathUtil.clamp( yDistance <= 0.2 ? 0.0 : (offset.getX() / -Reef.faceLength), 0.0, 1.0):
      MathUtil.clamp( yDistance <= 0.2 ? 0.0 : (offset.getX() / Reef.faceLength), 0.0, 1.0);

    //double shiftYT = MathUtil.clamp( yDistance <= 0.2 ? 0.0 : (offset.getX() / Reef.faceLength), 0.0, 1.0) ;

    
        DogLog.log("AutoScoreCommand/shiftXT" , shiftXT); 
        DogLog.log("AutoScoreCommand/shiftYT" , shiftYT);

        goal = reefLevel.get() == ReefLevel.L5 ? 
               goal.plus(new Transform2d(shiftXT * maxDistanceReefLineup , Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY()), new Rotation2d()))
              : goal.plus(new Transform2d(-shiftXT * maxDistanceReefLineup , Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY()), new Rotation2d()));

        DogLog.log("AutoScoreCommand/shiftedGoalPose", goal);
        
        return goal;
  }

   /** Get drive target. */
   private static Pose2d getDriveTargetStation(Supplier<Pose2d> robotPose, Supplier<Pose2d> goalPose) {

    //Todo: add logic for station
       return goalPose.get();
   }

  
  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

}
