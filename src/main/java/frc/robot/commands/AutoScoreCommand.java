package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoScoreCommand extends SequentialCommandGroup {
  public AutoScoreCommand(CommandSwerveDrivetrain drive, 
                          Supplier<Pose2d> robotPoseSupplier,
                         Supplier<Pose2d> goalPoseSupplier,
                         Supplier<ReefLevel> reefLevelSupplier,
                         Supplier<Boolean> isBackWordSupplier,
                         DoubleSupplier linearFF_X,
                         DoubleSupplier linearFF_Y,
                         DoubleSupplier omegaFF, Arm arm, IntSupplier reefTarget
                         ) {

        
    addCommands(     
      new ParallelCommandGroup(
        Autos.getAutoDriveCommandReef(drive, robotPoseSupplier, goalPoseSupplier, reefLevelSupplier,isBackWordSupplier, linearFF_X,linearFF_Y, omegaFF),
        new WaitUntilCommand(() -> robotPoseSupplier.get().minus(goalPoseSupplier.get()).getTranslation().getNorm() < 1).andThen(new ArmCommand(arm, reefTarget))
        )
    );
  }
    
}
