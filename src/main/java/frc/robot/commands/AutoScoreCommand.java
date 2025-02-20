package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoScoreCommand extends SequentialCommandGroup {
  public AutoScoreCommand(CommandSwerveDrivetrain drive, 
                          Supplier<Pose2d> robotPoseSupplier,
                         Supplier<Pose2d> goalPoseSupplier
                         ) {

        
    addCommands(     
        getAutoDriveCommand(drive, robotPoseSupplier, goalPoseSupplier)
        
    );
  }

  public static Command getAutoDriveCommand(
     CommandSwerveDrivetrain drive,
     Supplier<Pose2d> robotPoseSupplier,
     Supplier<Pose2d> goalPoseSupplier) {

        return new DriveToPose(drive, () -> getDriveTarget(()->drive.getState().Pose, ()->goalPoseSupplier.get()));
      } 


  /** Get drive target. */
  private static Pose2d getDriveTarget(Supplier<Pose2d> robotPose, Supplier<Pose2d> goalPose) {
    double maxDistanceReefLineup = 1.5;

    var robot = robotPose.get();
    var goal = goalPose.get();
    // Final line up
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());

    DogLog.log("AutoScoreCommand/goalPose" , goal);
    DogLog.log("AutoScoreCommand/robotPose" , robot);
    DogLog.log("AutoScoreCommand/xDistance" , xDistance);
    DogLog.log("AutoScoreCommand/yDistance" , yDistance);

    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
            0.0,
            1.0);
    double shiftYT = MathUtil.clamp(offset.getX() / Reef.faceLength, 0.0, 1.0);
    //  goal.transformBy(
    //     GeomUtil.toTransform2d(
    //         -shiftXT * maxDistanceReefLineup,
    //         Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY())));


    DogLog.log("AutoScoreCommand/shiftedGoalPose" , goal.plus(new Transform2d(-shiftXT * maxDistanceReefLineup, Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY()), new Rotation2d())));

    return goal.plus(new Transform2d(-shiftXT * maxDistanceReefLineup, Math.copySign(shiftYT * maxDistanceReefLineup * 0.8, offset.getY()), new Rotation2d()));

  }

    
}
