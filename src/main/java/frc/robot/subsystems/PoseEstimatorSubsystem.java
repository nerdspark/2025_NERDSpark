package frc.robot.subsystems;

import static frc.robot.Constants.Vision.kCameraNameFront;
import static frc.robot.Constants.Vision.kRobotToCamFront;
import static frc.robot.Constants.Vision.kCameraNameBack;
import static frc.robot.Constants.Vision.kRobotToCamBack;
import static frc.robot.Constants.Vision.kTagLayout;
import static frc.robot.Constants.Vision.USE_VISION;


import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
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



    // private Vision vision2 = new Vision(kCameraName, kRobotToCam);
    // private Vision vision3 = new Vision(kCameraName, kRobotToCam);
    // private Vision vision4 = new Vision(kCameraName, kRobotToCam);

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
             Optional<EstimatedRobotPose> visionEst = Optional.empty();
            visionEst = visionFront.getEstimatedRobotPose();
            visionEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = visionFront.getEstimationStdDevs();

                    SmartDashboard.putString("Adding Front visionEst ", getFomattedPose(est.estimatedPose.toPose2d()));
                    SmartDashboard.putNumber("Adding Front visionEst timestamp", Utils.fpgaToCurrentTime(est.timestampSeconds));
                    SmartDashboard.putNumber("Robot Timestamp ", Utils.getCurrentTimeSeconds());

                    printMatrixValues(estStdDevs);
                    driveTrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
                });

            visionEst = visionBack.getEstimatedRobotPose();
            visionEst.ifPresent(
                est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = visionBack.getEstimationStdDevs();
                
                SmartDashboard.putString("Adding Back visionEst ", getFomattedPose(est.estimatedPose.toPose2d()));
                
                
                driveTrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
            });

            // if(Robot.isSimulation() ) {
            //     visionFront.simulationPeriodic(driveTrain.getDriveTrainSimulationPose());
            //     SmartDashboard.putData("Debug Field Front", visionFront.getSimDebugField());
            //     visionBack.simulationPeriodic(driveTrain.getDriveTrainSimulationPose());
            //     SmartDashboard.putData("Debug Field Back", visionBack.getSimDebugField());
    
            // }
        }
        else {
            if (allNotifier != null) allNotifier.close();
        }

        if (getCurrentPose() != null) {
            field.setRobotPose(getCurrentPose());
            SmartDashboard.putData("Robot Pose in Field", field);
            SmartDashboard.putString("Robot Pose", getFomattedPose());
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
                SmartDashboard.putNumber("Stddev "+ i,curStdDevs.get(i, j));
            }
        }
    }
} 