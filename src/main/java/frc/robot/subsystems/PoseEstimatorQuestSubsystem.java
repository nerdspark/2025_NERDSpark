package frc.robot.subsystems;

import java.util.Optional;

import static frc.robot.Constants.Vision.USE_QUESTNAV;
import static frc.robot.Constants.Vision.USE_VISION;
import static frc.robot.Constants.Vision.kCameraNameBack;
import static frc.robot.Constants.Vision.kCameraNameFront;
import static frc.robot.Constants.Vision.kRobotToCamBack;
import static frc.robot.Constants.Vision.kRobotToCamFront;

import org.photonvision.EstimatedRobotPose;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav.NerdQuestNav;

public class PoseEstimatorQuestSubsystem extends SubsystemBase {
    
    public Vision visionFront;
    public Vision visionBack;
    private static Notifier allNotifier;
    private NerdQuestNav QuestNAV;
    StructPublisher<Pose2d> publisherQuest;
    private Field2d field = new Field2d();

    public PoseEstimatorQuestSubsystem(NerdQuestNav questNav) {
        if(USE_VISION) {
    
            this.visionFront = new Vision(kCameraNameFront, kRobotToCamFront);
            this.visionBack = new Vision(kCameraNameBack, kRobotToCamBack);
            this.QuestNAV = questNav;
                
            allNotifier = new Notifier(() -> {
                visionFront.run();
                visionBack.run();
            });
    
            allNotifier.setName("runAll");
            allNotifier.startPeriodic(0.02);

            publisherQuest = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose Quest", Pose2d.struct).publish();
        }
    }

    @Override
    public void periodic() {
        if (USE_QUESTNAV) {
            Optional<EstimatedRobotPose> visionEstFrontQuest  = visionFront.getEstimatedRobotPoseQuest();
            Optional<EstimatedRobotPose> visionEstBackQuest  = visionBack.getEstimatedRobotPoseQuest();

            visionEstFrontQuest.ifPresent(estQuest -> {                        
                QuestNAV.resetPose(estQuest.estimatedPose);
                //QuestNAV.hardReset(estQuest.estimatedPose);
            });

            visionEstBackQuest.ifPresent(estQuest -> {                        
                QuestNAV.resetPose(estQuest.estimatedPose);                    
                //QuestNAV.hardReset(estQuest.estimatedPose);  
            });

            if (QuestNAV.getRobotPose().isPresent() && QuestNAV.initializedPosition) {
                field.setRobotPose(QuestNAV.getRobotPose().get().toPose2d());
                SmartDashboard.putString("Quest Pose", getFomattedPose(QuestNAV.getRobotPose().get().toPose2d()));
                DogLog.log("PoseEstimator/Quest Pose", QuestNAV.getRobotPose().get().toPose2d());
                publisherQuest.set(QuestNAV.getRobotPose().get().toPose2d());
                SmartDashboard.putData("Robot Pose in Field", field);
            }
        }
    }

    private String getFomattedPose(Pose2d pose) {
        return String.format(
            "(%.3f, %.3f) %.2f degrees",
            pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
}
