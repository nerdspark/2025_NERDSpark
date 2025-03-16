// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Vision;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.util.ArmPoint;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command grabFromFunnel(Arm arm, Gripper gripper) {
    return new SequentialCommandGroup(
      new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7].withWristFlip(4.2).withWristTwist(-3.141).add(new Translation2d(3, 10)))
      .alongWith(new ArmCommandGripperPosition(gripper, () -> -0.09)).withTimeout(0.8),
      new ArmCommand(arm,  () -> ArmSetpoints.armSetPoints[7].withWristFlip(4.2).withWristTwist(-3.141))
        .alongWith(new ArmCommandGripperPosition(gripper, () -> -0.09)).withTimeout(0.4),
      new ArmCommandGripperForceClose(gripper).withTimeout(0.2), 
      new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7].withWristFlip(4.2).withWristTwist(-3.141).add(new Translation2d(3, 15))).withTimeout(0.8),
      new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7].withWristFlip(1.5).withWristTwist(-3.141).add(new Translation2d(3, 15))).withTimeout(1.0)
    ).withTimeout(4);
  }
  

  public static Command getAutoDriveCommandReef(
     CommandSwerveDrivetrain drive,
     Supplier<Pose2d> robotPoseSupplier,
     Supplier<Pose2d> goalPoseSupplier,
     Supplier<ReefLevel> reefLevelSupplier,
     Supplier<Boolean> isBackwardsSupplier,
     DoubleSupplier linearFF_X,
     DoubleSupplier linearFF_Y,
     DoubleSupplier omegaFF) {

       return new DriveToPose(drive, 
       () -> getDriveTargetReef(()->drive.getState().Pose, goalPoseSupplier, reefLevelSupplier, isBackwardsSupplier),
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

  public static Command getTransferCommand(Arm arm, Intake intake, Gripper gripper) {
    return intake.getRangeIntakeDetected() ? 
          new SequentialCommandGroup(
            new OpenGripperCommandStrong(gripper).until(() -> arm.finishedMoving && arm.wristFinishedMoving()), 
            new OpenGripperCommandStrong(gripper).alongWith(new IntakeCommandPower(intake, () -> IntakeConstants.transferPowerDeploy, () -> 0.0).until(() -> intake.finishedMovingToTransfer()).andThen(new WaitCommand(0.12))).withTimeout(0.65), 
            new ArmCommandGripperForceClose(gripper)
              .alongWith(new IntakeCommand(intake, () -> IntakeConstants.intakeTransferPosition, () -> IntakeConstants.transferPowerRollers))
                .withTimeout(0.2))
          .deadlineFor(new ArmCommandPathToPoint(arm, () -> 8))
          .andThen(new ArmCommandPathToPoint(arm, () -> 11).withTimeout(0.15)) : new ArmDefaultCommand(arm, () -> 7);
  }

  // public static Command getDropOffCommand(Arm arm, Gripper gripper) {

  //   return 
  //     new SequentialCommandGroup(
  //       new ArmCommandPathToPoint(arm, () -> new ArmPoint(arm.getArmPosition().plus(new Translation2d(8, new Rotation2d(arm.getWristFlipPosition() - Math.PI))), arm.getCurrentInBend(), arm.getWristFlipPosition(), arm.getWristTwistPosition())).withTimeout(0.5), 
  //       new ArmCommandGripper(gripper, () -> false).withTimeout(0.5));
  // }

  public static Command getDropReefOffCommand(Arm arm, Gripper gripper, IntSupplier setPointIndex) {
    return //new ArmCommandGripper(gripper, () -> false).withTimeout(0.15);
      new SequentialCommandGroup(
        new ArmCommandPathToPoint(arm, () -> ArmSetpoints.armSetPoints[setPointIndex.getAsInt()].add(new Translation2d(12, new Rotation2d(arm.getWristFlipPosition() + (arm.getWristFlipPosition() > (Math.PI*0.5) ? (Math.PI*0.5) : (-Math.PI*0.5)) ))))
          .alongWith(new WaitCommand(0.07).andThen(new ArmCommandGripper(gripper, () -> false))).withTimeout(0.15), 
        new ArmCommandPathToPoint(arm, () -> ArmSetpoints.armSetPoints[setPointIndex.getAsInt()].add(new Translation2d(10, new Rotation2d(ArmSetpoints.armSetPoints[setPointIndex.getAsInt()].position.getX() < 8 ? 0 : Math.PI)))).withTimeout(0.1)).onlyIf(() -> (setPointIndex.getAsInt() == 3 || setPointIndex.getAsInt() == 4)).andThen(new ArmCommandGripper(gripper, () -> false).withTimeout(0.30));
  }
  public static Command getDunkDropCommand(Arm arm, Gripper gripper, IntSupplier setPointIndex) {
  return 
      new ArmCommandPathToPoint(arm, () -> ArmSetpoints.armSetPoints[setPointIndex.getAsInt()].flipBy(ArmSetpoints.armSetPoints[setPointIndex.getAsInt()].position.getX() > 10 ? -1.5 : 1.5)).withTimeout(0.8)
        .alongWith(new WaitCommand(0.3).andThen(new ArmCommandGripper(gripper, () -> false).withTimeout(0.5))).withTimeout(1);
  }
  public static Command getDunkCommand(Arm arm, IntSupplier setPointIndex) {
    return 
        new ArmCommandPathToPoint(arm, () -> ArmSetpoints.armSetPoints[setPointIndex.getAsInt()].flipBy(ArmSetpoints.armSetPoints[setPointIndex.getAsInt()].position.getX() > 10 ? -1.5 : 1.5));
    }

  public static Command removeAlgaeCommand(Arm arm, Gripper gripper, boolean higherLevel) {
    return new ArmCommandPathToPoint(arm, () -> 6).alongWith(new ArmCommandGripperAutoClose(gripper, () -> false, () -> true));
  }
  public static Command funnelIntake(Arm arm, Gripper gripper) {
    return 
    new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7].add(new Translation2d(6, 7)).withWristFlip(3.9).withWristTwist(-3.141))
    .raceWith(new WaitCommand(0.7).andThen(new ArmCommandGripperFunnelPickup(gripper)))
    .andThen(new ArmCommand(arm, () -> ArmSetpoints.armSetPoints[7].withWristFlip(4.5).withWristTwist(-3.141)).withTimeout(0.2)
    .andThen(new ArmCommandGripper(gripper, () -> true).withTimeout(1)));

    // new ArmCommandPathToPoint(arm, () -> ArmSetpoints.armSetPoints[7].add(new Translation2d(3, 5)).withWristFlip(4.2).withWristTwist(-3.141))
    // .raceWith(new WaitCommand(0.7).andThen(new ArmCommandGripperFunnelPickup(gripper)))
    // .andThen(new ArmCommandPathToPoint(arm, () -> ArmSetpoints.armSetPoints[7].withWristFlip(4.5).withWristTwist(-3.141)).withTimeout(0.2)
    // .andThen(new ArmCommandGripper(gripper, () -> true).withTimeout(1)));
  }

  
  public static Command getAutoDriveCommandXY(
     CommandSwerveDrivetrain drive,
     Supplier<Pose2d> robotPoseSupplier,
     Supplier<Pose2d> goalPoseSupplier,
     Supplier<ReefLevel> reefLevelSupplier,
     Supplier<Boolean> isBackwardsSupplier) {

          return new DriveToPoseCommand(drive, robotPoseSupplier, 
                () -> getDriveTargetReef(()->drive.getState().Pose, ()->goalPoseSupplier.get(), ()->reefLevelSupplier.get(), isBackwardsSupplier),
                () -> drive.getState().Pose.getRotation());
          
  }

    /** Get drive target. */
  private static Pose2d getDriveTargetReef(Supplier<Pose2d> robotPose, Supplier<Pose2d> goalPose, Supplier<ReefLevel> reefLevel, Supplier<Boolean> isBackwardsSupplier) {
    double maxDistanceReefLineup = 1.5;
    var robot = robotPose.get();
    var goal = goalPose.get();
    // Final line up
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());

    double rotationDiff = offset.getRotation().getDegrees();

     
    boolean isBackwards = isBackwardsSupplier.get();

    double shiftXT = 0.0, shiftYT = 0.0;

       if(reefLevel.get() == ReefLevel.L4 || reefLevel.get() == ReefLevel.L3 || reefLevel.get() == ReefLevel.L2) {
   
          if(isBackwards){
            shiftXT = MathUtil.clamp((yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
            0.0,1.0);

            shiftYT = MathUtil.clamp( yDistance <= 0.2 ? 0.0 : (offset.getX() / Reef.faceLength), 0.0, 1.0);

           goal = goal.plus(new Transform2d(-shiftXT * maxDistanceReefLineup , Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY()), new Rotation2d()));

            
          }else{
            if(reefLevel.get() == ReefLevel.L4){
              shiftXT = MathUtil.clamp((yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
              0.0,1.0);

              shiftYT = MathUtil.clamp( yDistance <= 0.2 ? 0.0 : (offset.getX() / Reef.faceLength), 0.0, 1.0);

              goal = goal.plus(new Transform2d(-shiftXT * maxDistanceReefLineup , Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY()), new Rotation2d()));


            }else{
              shiftXT =    MathUtil.clamp(((yDistance) / (Reef.faceLength *2 )) - ((xDistance + 0.3) / (Reef.faceLength * 3)),
              0.0,1.0);

              shiftYT = MathUtil.clamp( yDistance <= 0.2 ? 0.0 : (offset.getX() / -Reef.faceLength), 0.0, 1.0);

              goal = goal.plus(new Transform2d(shiftXT * maxDistanceReefLineup , Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY()), new Rotation2d()));


            }
          }
         }else{
            shiftXT =    MathUtil.clamp(((yDistance) / (Reef.faceLength *2 )) - ((xDistance + 0.3) / (Reef.faceLength * 3)),
            0.0,1.0);

            shiftYT = MathUtil.clamp( yDistance <= 0.2 ? 0.0 : (offset.getX() / -Reef.faceLength), 0.0, 1.0);

            goal = goal.plus(new Transform2d(shiftXT * maxDistanceReefLineup , Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY()), new Rotation2d()));

          }
          if (Vision.DOGLOG_ENABLED) {
            DogLog.log("AutoScoreCommand/offset" , offset);   
        
            DogLog.log("AutoScoreCommand/goalPose" , goal);
            DogLog.log("AutoScoreCommand/robotPose" , robot);
            DogLog.log("AutoScoreCommand/xDistance" , xDistance);
            DogLog.log("AutoScoreCommand/yDistance" , yDistance);
            DogLog.log("AutoScoreCommand/offsetX" , offset.getX());
            DogLog.log("AutoScoreCommand/offsetY" , offset.getY());
            DogLog.log("AutoScoreCommand/rotationDiff" , rotationDiff);
          
        
        DogLog.log("AutoScoreCommand/shiftXT" , shiftXT); 
        DogLog.log("AutoScoreCommand/shiftYT" , shiftYT);

      
        DogLog.log("AutoScoreCommand/shiftedGoalPose", goal);
          }
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
