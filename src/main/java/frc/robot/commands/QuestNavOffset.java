package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.QuestNav5010;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class QuestNavOffset extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CommandSwerveDrivetrain drivetrainSubsystem;
    private static Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
    private static int _calculatedOffsetToRobotCenterCount = 0;
    private final SwerveRequest.ApplyRobotSpeeds driveToPoseRequest = new SwerveRequest.ApplyRobotSpeeds(); 
    private QuestNav5010 questnav = new QuestNav5010(new Transform3d());
    
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public QuestNavOffset(CommandSwerveDrivetrain drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumberArray("Quest Calculated Offset to Robot Center", new double[] {0, 0});
    }
    
    @Override
    public void execute() {
        SmartDashboard.putNumberArray("Quest Calculated Offset to Robot Center", new double[] {1, 0});
        Commands.repeatingSequence(
            Commands.run(
                () -> {
                    drivetrainSubsystem.setControl(driveToPoseRequest.withSpeeds(new ChassisSpeeds(0, 0, 0.314)));
                }
            ).withTimeout(2.0),
            Commands.runOnce(() -> {
                // Update current offset
                Translation2d offset = questnav.calculateOffsetToRobotCenter();
                
                _calculatedOffsetToRobotCenter = _calculatedOffsetToRobotCenter.times((double)_calculatedOffsetToRobotCenterCount / (_calculatedOffsetToRobotCenterCount + 1))
                    .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
                _calculatedOffsetToRobotCenterCount++;

                SmartDashboard.putNumberArray("Quest Calculated Offset to Robot Center", new double[] { _calculatedOffsetToRobotCenter.getX(), _calculatedOffsetToRobotCenter.getY() });

            })
        ).withTimeout(10.0);
        SmartDashboard.putNumberArray("Quest Calculated Offset to Robot Center", new double[] {2, 0});
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // ChassisSpeeds zeroChassisSpeeds = new ChassisSpeeds();
        // drivetrainSubsystem.setControl(driveToPoseRequest.withSpeeds(zeroChassisSpeeds));
        drivetrainSubsystem.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
        SmartDashboard.putNumberArray("Quest Calculated Offset to Robot Center", new double[] {1, 1});
    }
}