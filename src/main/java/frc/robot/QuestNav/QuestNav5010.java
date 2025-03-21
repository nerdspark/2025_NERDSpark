// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.QuestNav;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class QuestNav5010 {
    private boolean initializedPosition = false;
    private String networkTableRoot = "questnav";
    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private NetworkTable networkTable;
    private Transform3d robotToQuest;
    private Pose3d initPose = new Pose3d();
    private Transform3d softResetTransform = new Transform3d();
    private Pose3d softResetPose = new Pose3d();

    private IntegerEntry miso;
    private IntegerPublisher mosi;

    private IntegerSubscriber frameCount;
    private DoubleSubscriber timestamp;
    private FloatArraySubscriber position;
    private FloatArraySubscriber quaternion;
    private FloatArraySubscriber eulerAngles;
    private DoubleSubscriber battery;
    private double startTimestamp;

    private ChassisSpeeds velocity;
    private Pose3d previousPose;
    private double previousTime;

    private long previousFrameCount;
    
    private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
    private int _calculatedOffsetToRobotCenterCount = 0;

    private final SwerveRequest.ApplyRobotSpeeds driveToPoseRequest = new SwerveRequest.ApplyRobotSpeeds();
    private Boolean firstRun = true;
    private Distance softReseter = new Translation3d().getMeasureY();

    public enum QuestCommand {
        RESET(1);

        public final int questRequestCode;

        private QuestCommand(int command) {
            this.questRequestCode = command;
        }

        public int getQuestRequest() {
            return questRequestCode;
        }
    }

    public QuestNav5010(Transform3d robotToQuest) {
        super();
        this.robotToQuest = robotToQuest;
        setupNetworkTables(networkTableRoot);
        setupInitialTimestamp();
    }

    public QuestNav5010(Transform3d robotToQuest, String networkTableRoot) {
        super();
        this.robotToQuest = robotToQuest;
        this.networkTableRoot = networkTableRoot;
        setupNetworkTables(networkTableRoot);
        setupInitialTimestamp();
    }

    private void setupInitialTimestamp() {
        startTimestamp = timestamp.get();
    }


    private void setupNetworkTables(String root) {
        PubSubOption periodic = PubSubOption.periodic(0);
        networkTable = networkTableInstance.getTable(root);
        miso = networkTable.getIntegerTopic("miso").getEntry(0);
        mosi = networkTable.getIntegerTopic("mosi").publish();
        frameCount = networkTable.getIntegerTopic("frameCount").subscribe(0, periodic);
        timestamp = networkTable.getDoubleTopic("timestamp").subscribe(0.0, periodic);
        position = networkTable.getFloatArrayTopic("position").subscribe(new float[3], periodic);
        quaternion = networkTable.getFloatArrayTopic("quaternion").subscribe(new float[4]);
        eulerAngles = networkTable.getFloatArrayTopic("eulerAngles").subscribe(new float[3], periodic);
        battery = networkTable.getDoubleTopic("battery").subscribe(0.0);
    }

    public Translation3d getRawPosition() {
        return new Translation3d(position.get()[2], -position.get()[0], position.get()[2]);
    }

    private Translation3d rotateAxes(Translation3d raw, Rotation3d rotation) {
        return raw.rotateBy(rotation);
    }

    private Translation3d correctWorldAxis(Translation3d rawPosition) {
        return rotateAxes(rawPosition, robotToQuest.getRotation());
    }

    public Rotation3d getRawRotation() {
        float[] euler = eulerAngles.get();
        return new Rotation3d(Degrees.of(euler[2]), Degrees.of(euler[0]), Degrees.of(-euler[1]));
    }

    public Optional<Pose3d> getRobotPose() {
        if (RobotBase.isReal()) {
            Pose3d pose = new Pose3d(getPosition(), getRotation());
            return Optional.of(pose);
        } else {
            return Optional.empty();
        }
    }

    public Translation3d getProcessedPosition() {
        Translation3d hardResetTransformation = rotateAxes(correctWorldAxis(getRawPosition())
                .plus(robotToQuest.getTranslation())
                .plus(robotToQuest.getTranslation().times(-1).rotateBy(getRawRotation())),
                initPose.getRotation())
                .plus(initPose.getTranslation());
        return hardResetTransformation;
        
    }

    public Translation3d getPosition() {
        Translation3d hardResetTransform = getProcessedPosition();
        Translation3d softResetTransformation = rotateAxes(hardResetTransform.minus(softResetPose.getTranslation()), softResetTransform.getRotation()).plus(softResetPose.getTranslation()).plus(softResetTransform.getTranslation());
        if (DriverStation.isAutonomousEnabled()) {
            if (firstRun) {
                //SmartDashboard.putNumberArray("softReset", new Double[] {softResetTransformation.getX(), softResetTransformation.getY(), softResetTransformation.getZ()});
                softReseter = softResetTransformation.getMeasureY().times(2);
                firstRun = false;
            }
        }
        Distance softY = softResetTransformation.getMeasureY().minus(softReseter);
        return new Translation3d(softResetTransformation.getMeasureX(), softY.unaryMinus(), softResetTransformation.getMeasureZ());//.unaryMinus()
        //}
        //return softResetTransformation;
        //return new Translation3d(softResetTransformation.getMeasureX(), softResetTransformation.getMeasureY().unaryMinus(), softResetTransformation.getMeasureZ());
    }


    public Rotation3d getProcessedRotation() {
        return getRawRotation().plus(initPose.getRotation());
    }

    public Rotation3d getRotation() {
        // TODO: To support weird rotations/mountings of quest, implement world axis
        // rotation
       return getProcessedRotation().plus(softResetTransform.getRotation()).unaryMinus();
    }

    public double getConfidence() {
        if (RobotBase.isReal()) {
            return 0.01;
        } else {
            return Double.MAX_VALUE;
        }
    }

    public double getCaptureTime() {
        double t = RobotController.getFPGATime() / 1E6;
        SmartDashboard.putNumber("Quest Timestamp", t);
        return t;
    }

    public boolean isActive() {
        double t = timestamp.get();
        boolean simulation = RobotBase.isSimulation();
        boolean disabled = DriverStation.isDisabled();
        double frame = frameCount.get();
        double previousFrame = previousFrameCount;
        if (t == 0|| simulation || disabled) {
        return false;
        }
        previousFrameCount = frameCount.get();
        return initializedPosition;
    }

    public boolean processQuestCommand(QuestCommand command) {
        if (miso.get() == 99) {
            return false;
        }
        mosi.set(command.getQuestRequest());
        return true;
    }

    private void resetQuestPose() {
        processQuestCommand(QuestCommand.RESET);
    }

    public void softReset(Pose3d pose) {
        softResetTransform = new Transform3d(pose.getTranslation().minus(getProcessedPosition()), pose.getRotation().minus(getProcessedRotation()));
        softResetPose = new Pose3d(getProcessedPosition(), getProcessedRotation());
    }

    public void hardReset(Pose3d pose) {
        initPose = pose;
        resetQuestPose();
    }

    public void resetPose(Pose3d pose) {
        initializedPosition = true;
        softReset(pose);
        //hardReset(pose);
    }

    

    // @Override
    // public ProviderType getType() {
    //     return ProviderType.ENVIRONMENT_BASED;
    // }

    public void resetPose() {
        initializedPosition = true;
        resetQuestPose();
    }

    public void cleanUpQuestCommand() {
        if (miso.get() == 99) {
            mosi.set(0);
        }
    }

    public int fiducialId() {
        return 0;
    }

    private void updateVelocity() {
        if (previousPose == null) {
            previousPose = getRobotPose().get();
            previousTime = timestamp.get();
            return;
        }
        double currentTime = timestamp.get();
        double deltaTime = currentTime - previousTime;
        if (deltaTime == 0) {
            return;
        }
        velocity = new ChassisSpeeds(
                (getPosition().getX() - previousPose.getTranslation().getX()) / deltaTime,
                (getPosition().getY() - previousPose.getTranslation().getY()) / deltaTime,
                (getRotation().getZ() - previousPose.getRotation().getZ()) / deltaTime);
        previousTime = currentTime;
        previousPose = getRobotPose().get();

    }

    public ChassisSpeeds getVelocity() {
        if (null != velocity) {
            return velocity;
        }
        return new ChassisSpeeds();
    }

    // @Override
    // public void update() {
    //     if (RobotBase.isReal()) {
    //         cleanUpQuestCommand();
    //         updateVelocity();

    //         Pose2d currPose = getRobotPose().get().toPose2d();
    //         SmartDashboard.putNumberArray("Quest POSE", new double[] {
    //             currPose.getX(), currPose.getY(), currPose.getRotation().getDegrees()
    //         });

    //         ChassisSpeeds velocity = getVelocity();
    //         SmartDashboard.putNumberArray("Velocity", new double[] { velocity.vxMetersPerSecond,
    //                 velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond });
    //     }
    // }

    private Translation2d calculateOffsetToRobotCenter() {
        Pose3d currentPose = getRobotPose().get();
        Pose2d currentPose2d = currentPose.toPose2d();

        Rotation2d angle = currentPose2d.getRotation();
        Translation2d displacement = currentPose2d.getTranslation();

        double x = ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY()) / (2 * (1 - angle.getCos()));
        double y = ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY()) / (2 * (1 - angle.getCos()));

        return new Translation2d(x, y);
    }


    public Command determineOffsetToRobotCenter(CommandSwerveDrivetrain drivetrain) {
        return Commands.repeatingSequence(
            Commands.run(
            () -> {
                    drivetrain.setControl(driveToPoseRequest.withSpeeds(new ChassisSpeeds(0, 0, 0.314)));
                }
            ).withTimeout(0.5),
            Commands.runOnce(() -> {
                // Update current offset
                Translation2d offset = calculateOffsetToRobotCenter();
                
                _calculatedOffsetToRobotCenter = _calculatedOffsetToRobotCenter.times((double)_calculatedOffsetToRobotCenterCount / (_calculatedOffsetToRobotCenterCount + 1))
                    .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
                _calculatedOffsetToRobotCenterCount++;

                SmartDashboard.putNumberArray("Quest Calculated Offset to Robot Center", new double[] { _calculatedOffsetToRobotCenter.getX(), _calculatedOffsetToRobotCenter.getY() });

                }
            ).onlyIf(() -> getRotation().getMeasureZ().in(Degrees) > 30 || getRotation().getMeasureZ().in(Degrees) < 0));
    }

}
