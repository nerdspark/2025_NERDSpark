package frc.robot.subsystems;

import static frc.robot.Constants.Vision.kCameraNameFront;
import static frc.robot.Constants.Vision.kRobotToCamFront;
import static frc.robot.Constants.Vision.kCameraNameBack;
import static frc.robot.Constants.Vision.kRobotToCamBack;
import static frc.robot.Constants.Vision.kTagLayout;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class PoseEstimatorSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrain;
    private Vision visionFront ;
    private Vision visionBack ;

    // private Vision vision2 = new Vision(kCameraName, kRobotToCam);
    // private Vision vision3 = new Vision(kCameraName, kRobotToCam);
    // private Vision vision4 = new Vision(kCameraName, kRobotToCam);

    private Field2d field = new Field2d(); 
    
  
    // Simulation


    public PoseEstimatorSubsystem(CommandSwerveDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
        this.visionFront = new Vision(kCameraNameFront, kRobotToCamFront);
        this.visionBack = new Vision(kCameraNameBack, kRobotToCamBack);
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        Optional<EstimatedRobotPose> visionEst;
        visionEst = visionFront.getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = visionFront.getEstimationStdDevs();

                    driveTrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });

        visionEst = visionBack.getEstimatedGlobalPose();
        visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = visionBack.getEstimationStdDevs();

                driveTrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });


        if (getCurrentPose() != null) {
            field.setRobotPose(getCurrentPose());
            SmartDashboard.putData("Robot Pose in Field", field);
            SmartDashboard.putString("Robot Pose", getFomattedPose());
        }
        if(Robot.isSimulation()) {
            visionFront.simulationPeriodic(getCurrentPose());
            SmartDashboard.putData("Debug Field Front", visionFront.getSimDebugField());
            visionBack.simulationPeriodic(getCurrentPose());
            SmartDashboard.putData("Debug Field Back", visionBack.getSimDebugField());

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

    /**
     * Calculate the standard deviation of the x and y coordinates.
     *
     * @param poseEstimates The pose estimate
     * @param tagPosesSize The number of detected tag poses
     * @return The standard deviation of the x and y coordinates
     */

    /**
     * Calculate the standard deviation of the theta coordinate.
     *
     * @param poseEstimates The pose estimate
     * @param tagPosesSize The number of detected tag poses
     * @return The standard deviation of the theta coordinate
     *

    /**
     * Updates the inputs for AprilTag vision.
     *
     * @param estimator PhotonVisionRunnable estimator.
     * @param inputs The AprilTagVisionIOInputs object containing the inputs.
     */

}
