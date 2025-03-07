package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.config.SmartMotionConfigAccessor;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.util.CoralObject;
import frc.robot.subsystems.Gyro;


public class PoseEstimatorSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;
    private Vision visionFront;
    private Vision visionBack;
    private static Notifier allNotifier;

    //private final Pigeon2 gyro = new Pigeon2(TunerConstants.kPigeonId);
    //private PIDController GyroPID = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
    //public double targetAngle = 0;
    //private Rotation2d gyroResetAngle = new Rotation2d();

    private static Gyro gyro = new Gyro();
    private static List<CoralObject> corals = new ArrayList<>();
       
        private Field2d field = new Field2d(); 
          
        // Simulation
    
        public PoseEstimatorSubsystem(CommandSwerveDrivetrain driveTrain) {
            this.driveTrain = driveTrain;
            if(USE_VISION) {
    
                this.visionFront = new Vision(kCameraNameFront, kRobotToCamFront);
                this.visionBack = new Vision(kCameraNameBack, kRobotToCamBack);
    
                allNotifier = new Notifier(() -> {
                    visionFront.run();
                    visionBack.run();
                });
    
                allNotifier.setName("runAll");
                allNotifier.startPeriodic(0.02);  
              
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

            //Coral pose code 

            Pose2d coralPose = getCoralPose();

            if (visionFront.hasTarget()) {
                double hb = visionFront.getHB();
                CoralObject coral = new CoralObject(coralPose, hb, 0);
                coral.setCoralDistance(coral.calcDistance(coral.getPose()));
                SmartDashboard.putNumber("hb", coral.getHB());
                corals.add(coral);
            }

            if (corals.size() > 2) {
                int size = corals.size() - 2;
                Pose2d coralPoseLast = corals.get(size).getPose();
                double coralXLast = coralPoseLast.getX();
                double coralYLast = coralPoseLast.getY();
                SmartDashboard.putNumber("coralXLast", coralXLast);
                SmartDashboard.putNumber("coralYLast", coralYLast);

                SmartDashboard.putNumber("hbLast", corals.get(size).getHB());
            }

            
            
            SmartDashboard.putNumber("coralX", coralPose.getX());
            SmartDashboard.putNumber("coralY", coralPose.getY());
            SmartDashboard.putNumber("coralOrientation", coralPose.getRotation().getDegrees());

            

            SmartDashboard.putNumber("pigeon", gyro.getGyro().getDegrees());
            
        }
        else {
            if (allNotifier != null) allNotifier.close();
        }

        if (getCurrentPose() != null) {
            field.setRobotPose(getCurrentPose());
            // field.getObject("VisionEstimation").setPoses();

            SmartDashboard.putData("Robot Pose in Field", field);
            DogLog.log("PoseEstimator/Pose", getCurrentPose());
            DogLog.log("PoseEstimator/Formatted Pose", getFomattedPose());            

        }
        SmartDashboard.putBoolean("tV", visionFront.hasTarget());              
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

    public Pose2d getCoralPose() {
        Rotation2d yaw = gyro.getGyro();

        Pose2d pose = getCurrentPose();
        double poseX = pose.getX();
        double poseY = pose.getY();

        SmartDashboard.putNumber("poseX", poseX);
        SmartDashboard.putNumber("poseY", poseY);

        double tx = visionFront.getTx();
        double ty = visionFront.getTy();
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
        double widthAtParallel = 2.76016 * boundingHeight;
        double theta = 0.0;
        double threshTheta = Math.atan(boundingHeight / widthAtParallel);
        double thresh = widthAtParallel * Math.cos(threshTheta) + boundingHeight * Math.cos(Math.PI / 2 - threshTheta);
        if (boundingHeight > boundingWidth) {               
            //distance = (Constants.Vision.kCoralCenterUprightHeight - kLimeLightHeight) / Math.tan((Constants.Vision.kLimeLightAOD+ty) * (Math.PI / 180)) / Math.cos(tx * Math.PI / 180);
            SmartDashboard.putString("orientation", "upright");
        } else if (boundingHeight <= boundingWidth && boundingHeight != 0.0) {
            distance = (Constants.Vision.kCoralCenterFallenHeight - kLimeLightHeight) / Math.tan((Constants.Vision.kLimeLightAOD+ty) * (Math.PI / 180)) / Math.cos(tx * Math.PI / 180);
            SmartDashboard.putString("orientation", "fallen");
            
            theta = (Math.acos(boundingWidth / Math.sqrt(widthAtParallel * widthAtParallel + boundingHeight * boundingHeight)) + Math.atan(boundingHeight / widthAtParallel)) * 180 / Math.PI;
            SmartDashboard.putNumber("a", theta);
            SmartDashboard.putNumber("ratio1", boundingWidth / Math.sqrt(widthAtParallel * widthAtParallel + boundingHeight * boundingHeight));
            SmartDashboard.putNumber("widthAtParallel", widthAtParallel);
            if (boundingWidth > thresh) {
                theta = 2 * threshTheta - ((Math.acos(boundingWidth / Math.sqrt(widthAtParallel * widthAtParallel + boundingHeight * boundingHeight)) + Math.atan(boundingHeight / widthAtParallel)));
            } else {
                theta = ((Math.acos(boundingWidth / Math.sqrt(widthAtParallel * widthAtParallel + boundingHeight * boundingHeight)) + Math.atan(boundingHeight / widthAtParallel)));
            }
        } else {
            distance = 0.0;
            SmartDashboard.putString("orientation", "");
        }
        SmartDashboard.putNumber("a", theta);
        //SmartDashboard.putNumber("thresh", thresh);

        if (distance > 0.) {
            Rotation2d coralOrientation = new Rotation2d(theta);
            Pose2d coralPose = new Pose2d(distance * Math.sin((yaw.getDegrees()+tx) * (Math.PI / 180)) + Constants.Vision.kLimeLightXOffset + poseX, distance * Math.cos((yaw.getDegrees()+tx) * (Math.PI / 180)) + Constants.Vision.kLimeLightYOffset + poseY, coralOrientation);
            SmartDashboard.putNumber("distance", distance);
            return coralPose;
        } else {
            return new Pose2d(0,0, new Rotation2d(0.0));
        }
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