package frc.robot.subsystems;

import static frc.robot.Constants.Vision.USE_VISION;
import static frc.robot.Constants.Vision.kCameraNameBack;
import static frc.robot.Constants.Vision.kCameraNameFront;
import static frc.robot.Constants.Vision.kLimeLightHeight;
import static frc.robot.Constants.Vision.kRobotToCamBack;
import static frc.robot.Constants.Vision.kRobotToCamFront;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.CoralArrayManager;
import frc.robot.util.CoralObject;
import frc.robot.QuestNav.NerdQuestNav;


public class PoseEstimatorSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;
    public Vision visionFront;
    public Vision visionBack;
    private static Notifier allNotifier;
    private NerdQuestNav QuestNAV;


    Pose2d robotPose2d = new Pose2d();
    StructPublisher<Pose2d> publisher;
    StructPublisher<Pose2d> publisherQuest;


    //private final Pigeon2 gyro = new Pigeon2(TunerConstants.kPigeonId);
    //private PIDController GyroPID = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
    //public double targetAngle = 0;
    //private Rotation2d gyroResetAngle = new Rotation2d();

    private static List<CoralObject> corals = new ArrayList<>();
    private static CoralArrayManager coralManager = new CoralArrayManager();
    static boolean coralInRange = false;
       
        private Field2d field = new Field2d(); 
          
        // Simulation
    
        public PoseEstimatorSubsystem(CommandSwerveDrivetrain driveTrain, NerdQuestNav questNav) {
            this.driveTrain = driveTrain;
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

                publisher = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose AdvScope", Pose2d.struct).publish();

                publisherQuest = NetworkTableInstance.getDefault()
                .getStructTopic("Robot Pose Quest", Pose2d.struct).publish();
              
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
             
                driveTrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
            
                });

                if(Constants.Vision.QUEST_ENABLED){
                    Optional<EstimatedRobotPose> visionEstFrontQuest  = visionFront.getEstimatedRobotPoseQuest();
                    Optional<EstimatedRobotPose> visionEstBackQuest  = visionBack.getEstimatedRobotPoseQuest();

                    visionEstFrontQuest.ifPresent(
                    estQuest -> {                        
                        QuestNAV.resetPose(estQuest.estimatedPose);
                        //QuestNAV.hardReset(estQuest.estimatedPose);
                    });

                    visionEstBackQuest.ifPresent(
                        estQuest -> {                        
                            QuestNAV.resetPose(estQuest.estimatedPose);                    
                            //QuestNAV.hardReset(estQuest.estimatedPose);  
                    });
                    
                }

            if(Robot.isSimulation() ) {
                 visionFront.simulationPeriodic(getCurrentPose());
                SmartDashboard.putData("Debug Field Front", visionFront.getSimDebugField());
                 visionBack.simulationPeriodic(getCurrentPose());
                SmartDashboard.putData("Debug Field Back", visionBack.getSimDebugField());
    
            }

            if(visionEstFront.isPresent()) {
               DogLog.log("PoseEstimator/VisionEst", visionEstFront.get().estimatedPose.toPose2d());
            }

            if(visionEstBack.isPresent()) {
                DogLog.log("PoseEstimator/VisionEst", visionEstBack.get().estimatedPose.toPose2d());
            }   

            //coral code
            
            if (Constants.Vision.USE_LIMELIGHT) {

            // CoralObject newCoral = newCoral();
            // if (!newCoral.getIgnored() && visionFront.hasTarget()) {
            //     corals.add(newCoral); 
            // }

            

            corals = coralArrayUpdateReturn();
            SmartDashboard.putNumber("size", corals.size());
            SmartDashboard.putBoolean("targeting", Constants.Vision.kCoralTargeted);
            coralInRange = coralInRange();

            

            if (corals.size() > 0) {
                Pose2d coralPose = corals.get(corals.size() - 1).getPose();
                SmartDashboard.putString("coralPose", getFomattedPose(coralPose));
            }
            
            // if (corals.size() > 0) {
            //     int size = corals.size();
            //     CoralObject lastCoral = corals.get(size - 1);
            //     Pose2d lastCoralPose = lastCoral.getPose();
            //     SmartDashboard.putNumber("lastCoralX", lastCoralPose.getX());
            //     SmartDashboard.putNumber("lastCoralY", lastCoralPose.getY());
            //     SmartDashboard.putNumber("size", size);
            // }

            // Pose2d coralPose = newCoral.getPose();
            // SmartDashboard.putNumber("coralX", coralPose.getX());
            // SmartDashboard.putNumber("coralY", coralPose.getY());
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

            // SmartDashboard.putData("Robot Pose in Field", field);
            SmartDashboard.putString("Formatted Pose", getFomattedPose());

            DogLog.log("PoseEstimator/ODO+Vision Pose", getCurrentPose());
            DogLog.log("PoseEstimator/ODO+Vision Formatted Pose", getFomattedPose()); 
            
            if(Robot.isReal() && QuestNAV.getRobotPose().isPresent()) {
                //field.setRobotPose(QuestNAV.getRobotPose().get().toPose2d());
                SmartDashboard.putString("Quest Pose", getFomattedPose(QuestNAV.getRobotPose().get().toPose2d()));
                DogLog.log("PoseEstimator/Quest Pose", QuestNAV.getRobotPose().get().toPose2d());
                publisherQuest.set(QuestNAV.getRobotPose().get().toPose2d());
                SmartDashboard.putData("Robot Pose in Field", field);
            }

        }
        SmartDashboard.putBoolean("tV", visionFront.hasTarget()); 
        SmartDashboard.putBoolean("Coral In Range", coralInRange());  
        SmartDashboard.putBoolean("Coral In List", coralInList());     
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

    public CoralObject newCoral() {
        //Rotation2d yaw = gyro.getGyro();
        //Pose2d pose = new Pose2d(0.0,0.0,yaw);
        Pose2d pose = getCurrentPose();
        double poseX = pose.getX();
        double poseY = pose.getY();
        Rotation2d yaw = pose.getRotation();

        SmartDashboard.putNumber("poseX", poseX);
        SmartDashboard.putNumber("poseY", poseY);
        SmartDashboard.putNumber("yaw", -yaw.getDegrees());

        double tx = visionFront.getTx();
        double ty = visionFront.getTy();
        double hb = visionFront.getHB();
        boolean upfall = false;
        boolean ignored = false;
        boolean targeted = false;

        Translation2d offset = new Translation2d();
        //Translation2d offset = new Translation2d(-0.90, new Rotation2d((-yaw.getDegrees() + tx) * Math.PI / 180));

        //DriverStation.getMatchTime();

        double boundingHeight = 0.0;
        double boundingWidth = 0.0;       

        double[] xys = visionFront.getCoordinates();
        if (xys.length != 0) { //debug
            boundingHeight = xys[5] - xys[3];
            boundingWidth = xys[2] - xys[0];
            SmartDashboard.putNumber("boundingHeight", boundingHeight);
            SmartDashboard.putNumber("boundingWidth", boundingWidth);
        }

        double distance = 0.0;
        double theta = 0.0;
        if (boundingHeight > boundingWidth) {               
            upfall = false;
            ignored = true;
        } else if (boundingHeight <= boundingWidth && boundingHeight != 0.0) {
            distance = (Constants.Vision.kCoralCenterFallenHeight - kLimeLightHeight) / Math.tan((Constants.Vision.kLimeLightAOD+ty) * (Math.PI / 180)) / Math.cos((tx) * Math.PI / 180);
            upfall = true;
            ignored = false;
        } else {
            distance = 0.0;
            SmartDashboard.putString("orientation", "");
            ignored = true;
        }
        if (distance > 0.0) {
            Rotation2d coralOrientation = new   Rotation2d(theta);
            Pose2d coralPose = new Pose2d(distance * Math.cos((yaw.getDegrees()-tx) * (Math.PI / 180)) + poseX + offset.getX(), 
                                          distance * Math.sin((yaw.getDegrees()-tx) * (Math.PI / 180)) + poseY + offset.getY(), 
                                          yaw);
            //Pose2d coralPose = new Pose2d(2 + offset.getX(), 2 + offset.getY(), yaw);
            SmartDashboard.putNumber("distance", distance);
            ignored = false;
            CoralObject newCoral = new CoralObject(coralPose, hb, distance, upfall, targeted, ignored);
            return newCoral;
        } else {
            Pose2d zeroed = new Pose2d(0,0, new Rotation2d(0.0));
            ignored = true;
            CoralObject newCoral = new CoralObject(zeroed, hb, distance, upfall, targeted, ignored);
            // Pose2d coralPose = new Pose2d(2 + offset.getX(), 2 + offset.getY(), yaw);
            // SmartDashboard.putNumber("distance", distance);
            // ignored = false;
            // CoralObject newCoral = new CoralObject(coralPose, hb, distance, upfall, targeted, ignored);
            return newCoral;
        }
    }

    public List<CoralObject> coralArrayUpdateReturn() {
        if (!Constants.Vision.kCoralTargeted) {
            CoralObject newCoral = newCoral();
            double hb = visionFront.getHB();
            double fps = visionFront.getFPS();
            corals.add(newCoral);
            coralManager.distanceAndYawUpdate(corals, getCurrentPose());
            // coralManager.expiryFilter(corals, hb, fps);
            // coralManager.displacementFilter(corals);
            //coralManager.possibilityFilter(corals);
            return corals;
        } else {
            return coralManager.selectCoral(corals);
        }
    }

    public boolean coralInRange() {
        coralInRange = coralManager.getCoralInRange(corals, getCurrentPose());
        return coralInRange; 
    }

    public boolean coralInList() {
        return (corals.size() > 0);
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