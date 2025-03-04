package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;


import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
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
import frc.robot.Robot;


public class PoseEstimatorSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;
    private Vision visionFront ;
    private Vision visionBack ;
    private static Notifier allNotifier;
   
       
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

            //if (visionFront.getObjectClass()=="algae"){
            Pose2d algaePose = getCoralPose();
            
            SmartDashboard.putNumber("algaeX", algaePose.getX());
            SmartDashboard.putNumber("algaeY", algaePose.getY());
               // }
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
        //SmartDashboard.putString("class", visionFront.getObjectClass());
              
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
    
        Pose2d pose = getCurrentPose();
        double poseX = pose.getX();
        double poseY = pose.getY();
        Rotation2d gyro = new Rotation2d(0);
        //Rotation2d gyro = new Rotation2d(visionFront.getBotPose()[6]);
                double tx = visionFront.getTx();
                double ty = visionFront.getTy();

        //double tx = 0;
        //double ty = -20.0;
                
        double distance = (kAlgaeCenterHeight - kLimeLightHeight) / Math.tan((30+ty) * (Math.PI / 180)) / Math.cos(tx * Math.PI / 180) + 0.2032;
        Pose2d coralPose = new Pose2d(distance * Math.sin((gyro.getDegrees()+tx) * (Math.PI / 180)) + poseX, distance * Math.cos((gyro.getDegrees()+tx) * (Math.PI / 180)) + poseY, gyro);
        SmartDashboard.putNumber("distance", distance);
        return coralPose;
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