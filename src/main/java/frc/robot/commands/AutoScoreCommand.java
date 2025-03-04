package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoScoreCommand extends SequentialCommandGroup {
  public AutoScoreCommand(CommandSwerveDrivetrain drive, 
                          Supplier<Pose2d> robotPoseSupplier,
                         Supplier<Pose2d> goalPoseSupplier,
                         Supplier<ReefLevel> reefLevelSupplier,
                         DoubleSupplier linearFF_X,
                         DoubleSupplier linearFF_Y,
                         DoubleSupplier omegaFF
                         ) {

        
    addCommands(     
      Autos.getAutoDriveCommandReef(drive, robotPoseSupplier, goalPoseSupplier, reefLevelSupplier,linearFF_X,linearFF_Y, omegaFF)
        
    );
  }
    
}
