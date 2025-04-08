package frc.robot.subsystems;

import static frc.robot.Constants.Vision.USE_VISION;
import static frc.robot.Constants.Vision.kCameraNameBack;
import static frc.robot.Constants.Vision.kCameraNameFront;
import static frc.robot.Constants.Vision.kRobotToCamBack;
import static frc.robot.Constants.Vision.kRobotToCamFront;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.QuestNav.NerdQuestNav;


public class PoseEstimatorSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;
    public Vision visionFront;
    public Vision visionBack;
    private static Notifier allNotifier;
    private NerdQuestNav QuestNAV;


    Pose2d robotPose2d = new Pose2d();
    StructPublisher<Pose2d> publisher;


    //private final Pigeon2 gyro = new Pigeon2(TunerConstants.kPigeonId);
    //private PIDController GyroPID = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
    //public double targetAngle = 0;
    //private Rotation2d gyroResetAngle = new Rotation2d();

       
    private Field2d field = new Field2d(); 
          
        // Simulation
    
        public PoseEstimatorSubsystem(CommandSwerveDrivetrain driveTrain) {
            this.driveTrain = driveTrain;
            if(USE_VISION) {
    
                this.visionFront = new Vision(kCameraNameFront, kRobotToCamFront, driveTrain);
                this.visionBack = new Vision(kCameraNameBack, kRobotToCamBack, driveTrain);
                QuestNAV = new NerdQuestNav(new Transform3d(0, 0, 0, new Rotation3d(Rotation2d.fromDegrees(90))));
    
                allNotifier = new Notifier(() -> {
                    visionFront.run();
                    visionBack.run();
                });
    
                allNotifier.setName("runAll");
                allNotifier.startPeriodic(0.02);

                publisher = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose AdvScope", Pose2d.struct).publish();
              
        }        

    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        if(USE_VISION) {
             Optional<EstimatedRobotPose> visionEstFront  = visionFront.getEstimatedRobotPose();
          
            visionEstFront.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = visionFront.getEstimationStdDevs();
                    SignalLogger.writeBoolean("front refreshed", true);

                    // printMatrixValues(estStdDevs);
                    driveTrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);


                });    

            Optional<EstimatedRobotPose> visionEstBack  = visionBack.getEstimatedRobotPose();

            visionEstBack = visionBack.getEstimatedRobotPose();
            visionEstBack.ifPresent(
                est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = visionBack.getEstimationStdDevs();               
                SignalLogger.writeBoolean("front refreshed", true);

                driveTrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);

                });

                if(Constants.Vision.QUEST_ENABLED){
                    Optional<EstimatedRobotPose> visionEstFrontQuest  = visionFront.getEstimatedRobotPoseQuest();
                    Optional<EstimatedRobotPose> visionEstBackQuest  = visionBack.getEstimatedRobotPoseQuest();

                    visionEstFrontQuest.ifPresent(
                    estQuest -> {                        
                        QuestNAV.resetPose(estQuest.estimatedPose);                    

                });

                    visionEstBackQuest.ifPresent(
                        estQuest -> {                        
                            QuestNAV.resetPose(estQuest.estimatedPose);                    

                    });

                      
                    
                }

            if(Robot.isSimulation() ) {
                 visionFront.simulationPeriodic(getCurrentPose());
                // SmartDashboard.putData("Debug Field Front", visionFront.getSimDebugField());
                 visionBack.simulationPeriodic(getCurrentPose());
                // SmartDashboard.putData("Debug Field Back", visionBack.getSimDebugField());
    
            }

            if(visionEstFront.isPresent() && Constants.Vision.DOGLOG_ENABLED) {
               DogLog.log("PoseEstimator/VisionEst", visionEstFront.get().estimatedPose.toPose2d());
            }

            if(visionEstBack.isPresent() && Constants.Vision.DOGLOG_ENABLED) {
                DogLog.log("PoseEstimator/VisionEst", visionEstBack.get().estimatedPose.toPose2d());
             }
             
        }
        else {
            if (allNotifier != null) allNotifier.close();
        }

        if (getCurrentPose() != null) {
            field.setRobotPose(getCurrentPose());
            robotPose2d = getCurrentPose();
            publisher.set(robotPose2d);

            // field.getObject("VisionEstimation").setPoses();

            SmartDashboard.putData("Robot Pose in Field", field);
            SmartDashboard.putString("Formatted Pose", getFomattedPose());

            DogLog.log("PoseEstimator/ODO+Vision Pose", getCurrentPose());
            DogLog.log("PoseEstimator/ODO+Vision Formatted Pose", getFomattedPose()); 
            
            if(Robot.isReal() && QuestNAV.getRobotPose().isPresent()) {
                field.setRobotPose(QuestNAV.getRobotPose().get().toPose2d());
                SmartDashboard.putString("Quest Pose", getFomattedPose(QuestNAV.getRobotPose().get().toPose2d()));
                DogLog.log("PoseEstimator/Quest Pose", QuestNAV.getRobotPose().get().toPose2d());
            }

        }
    }
    

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    private String getFomattedPose(Pose2d pose) {
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return driveTrain.getState().Pose;
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        //driveTrain.seedFieldRelative(newPose);
        driveTrain.resetPose(newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    public void printMatrixValues(Matrix<N3, N1> curStdDevs) {
        
        for (int i = 0; i < curStdDevs.getNumRows(); i++) {
            for (int j = 0; j < curStdDevs.getNumCols(); j++) {
                DogLog.log("Stddev "+ i,curStdDevs.get(i, j));
            }
        }
    }
}