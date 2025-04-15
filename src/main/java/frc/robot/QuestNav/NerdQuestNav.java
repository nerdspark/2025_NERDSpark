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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class NerdQuestNav {
    public boolean initializedPosition = false;
    public static boolean isActive = false;
    private String networkTableRoot = "questnav";
    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private NetworkTable networkTable;
    private Transform3d robotToQuest;
    private Pose3d initPose = new Pose3d();
    private Transform3d softResetTransform = new Transform3d();
    private Pose3d softResetPose = new Pose3d();
    private double xScale = 1.0;

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
    private final double TIMESTAMP_DELAY = 0.002;

    private long previousFrameCount;

    private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
    private int _calculatedOffsetToRobotCenterCount = 0;

    private final SwerveRequest.ApplyRobotSpeeds m_ApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /** Subscriber for heartbeat requests */
    private DoubleSubscriber heartbeatRequestSub;
    /** Publisher for heartbeat responses */
    private DoublePublisher heartbeatResponsePub;
    /** Last processed heartbeat request ID */
    private double lastProcessedHeartbeatId = 0;

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

    public NerdQuestNav(Transform3d robotToQuest) {
        super();
        this.robotToQuest = robotToQuest;
        setupNetworkTables(networkTableRoot);
        setupInitialTimestamp();
    }

    public NerdQuestNav(Transform3d robotToQuest, String networkTableRoot) {
        super();
        this.robotToQuest = robotToQuest;
        this.networkTableRoot = networkTableRoot;
        setupNetworkTables(networkTableRoot);
        setupInitialTimestamp();
    }

    /** Process heartbeat requests from Quest and respond with the same ID */
    public void processHeartbeat() {
        double requestId = heartbeatRequestSub.get();
        // Only respond to new requests to avoid flooding
        if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
            heartbeatResponsePub.set(requestId);
            lastProcessedHeartbeatId = requestId;
        }
    }

    private void setupInitialTimestamp() {
        startTimestamp = timestamp.get();
    }

    private void setupNetworkTables(String root) {
        //PubSubOption periodic = PubSubOption.periodic(0);
        //PubSubOption sPeriodic = PubSubOption.periodic(30);
        networkTable = networkTableInstance.getTable(root);
        miso = networkTable.getIntegerTopic("miso").getEntry(0);
        mosi = networkTable.getIntegerTopic("mosi").publish();
        frameCount = networkTable.getIntegerTopic("frameCount").subscribe(0);
        timestamp = networkTable.getDoubleTopic("timestamp").subscribe(0.0);
        position = networkTable.getFloatArrayTopic("position").subscribe(new float[3]);
        quaternion = networkTable.getFloatArrayTopic("quaternion").subscribe(new float[4]);
        eulerAngles = networkTable.getFloatArrayTopic("eulerAngles").subscribe(new float[3]);
        battery = networkTable.getDoubleTopic("battery").subscribe(0.0);
        heartbeatRequestSub = networkTable.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
        heartbeatResponsePub = networkTable.getDoubleTopic("heartbeat/robot_to_quest").publish();
    }

    public Translation3d getRawPosition() {
        return new Translation3d(position.get()[2], -position.get()[0] * xScale, position.get()[1]);
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

    // @Override
    // public List<PoseObservation> getObservations() {
    //     List<PoseObservation> observations = new ArrayList<>();
    //     double calib = getConfidence();
    //     if (isActive) {
    //         observations.add(new PoseObservation(getCaptureTime(), 0, 0, VecBuilder.fill(calib, calib, calib*0.01),
    //                 new Pose3d(getPosition(), getRotation())));
    //     }
    //     return observations;
    // }

    public Translation3d getProcessedPosition() {
        Translation3d correctedWorldAxis = correctWorldAxis(getRawPosition());
        Translation3d offsetCorrection = correctedWorldAxis
                .plus(robotToQuest.getTranslation())
                .plus(robotToQuest.getTranslation().times(-1).rotateBy(new Rotation3d(0, 0, getRawRotation().getZ())));
        Translation3d rotatedAxis = rotateAxes(offsetCorrection, initPose.getRotation());
        Translation3d hardResetTransformation = rotatedAxis.plus(initPose.getTranslation());
        return hardResetTransformation;
    }

    public Translation3d getPosition() {
        Translation3d hardResetTransform = getProcessedPosition();
        Translation3d softResetTransformation = rotateAxes(hardResetTransform.minus(softResetPose.getTranslation()),
                softResetTransform.getRotation()).plus(softResetPose.getTranslation())
                .plus(softResetTransform.getTranslation());
        return softResetTransformation;
    }

    public Rotation3d getProcessedRotation() {
        return getRawRotation().plus(initPose.getRotation());
    }

    public Rotation3d getRotation() {
        return getProcessedRotation().plus(softResetTransform.getRotation());
    }

    public double getConfidence() {
        if (RobotBase.isReal()) {
            return 0.01;
        } else {
            return Double.MAX_VALUE;
        }
    }

    public double getCaptureTime() {
        double t = RobotController.getFPGATime() / 1E6 - TIMESTAMP_DELAY;
        SmartDashboard.putNumber("Quest Timestamp", t);
        updateFrameCount();
        return t;
    }

    public boolean isActive() {
        double t = timestamp.get();
        boolean simulation = RobotBase.isSimulation();
        double frame = frameCount.get();
        double previousFrame = previousFrameCount;
        if (t == 0 || simulation || previousFrame == frame) {
            isActive = false;
            return false;
        }
        isActive = true;
        return initializedPosition;
    }

    public void updateFrameCount() {
        previousFrameCount = frameCount.get();
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
        softResetTransform = new Transform3d(pose.getTranslation().minus(getProcessedPosition()),
            pose.getRotation().minus(getProcessedRotation()));
        softResetPose = new Pose3d(getProcessedPosition(), getProcessedRotation());
    }

    public void hardReset(Pose3d pose) {
        initPose = pose;
        resetQuestPose();
    }

    public static Trigger isQuestOn() {
        return new Trigger(() -> isActive);
    }

    public void resetPose(Pose3d pose) {
        initializedPosition = true;
        softReset(pose);
    }

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

    // private void updateVelocity() {
    //     if (previousPose == null) {
    //         previousPose = getRobotPose().get();
    //         previousTime = timestamp.get();
    //         return;
    //     }
    //     double currentTime = timestamp.get();
    //     double deltaTime = currentTime - previousTime;
    //     if (deltaTime == 0) {
    //         return;
    //     }
    //     velocity = new ChassisSpeeds(
    //             (getPosition().getX() - previousPose.getTranslation().getX()) / deltaTime,
    //             (getPosition().getY() - previousPose.getTranslation().getY()) / deltaTime,
    //             (getRotation().getZ() - previousPose.getRotation().getZ()) / deltaTime);
    //     previousTime = currentTime;
    //     previousPose = getRobotPose().get();

    // }

    // public ChassisSpeeds getVelocity() {
    //     if (null != velocity) {
    //         return velocity;
    //     }
    //     return new ChassisSpeeds();
    // }

    // @Override
    // public void update() {
    //     if (RobotBase.isReal()) {
    //         cleanUpQuestCommand();
    //         updateVelocity();
    //         SmartDashboard.putBoolean("Reset Pose", false);

    //         Pose2d currPose = getRobotPose().get().toPose2d();
    //         SmartDashboard.putNumberArray("Quest POSE", new double[] {
    //                 currPose.getX(), currPose.getY(), currPose.getRotation().getDegrees()
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

        double x = ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
                / (2 * (1 - angle.getCos()));
        double y = ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
                / (2 * (1 - angle.getCos()));

        return new Translation2d(x, y);
    }

    public Command determineOffsetToRobotCenter(CommandSwerveDrivetrain drivetrain, double speed) {
        resetPose();
        return Commands.repeatingSequence(
            Commands.run(() -> {
                drivetrain.setControl(
                    m_ApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0,0, speed)));
            }, drivetrain).withTimeout(0.5),
                
            Commands.runOnce(() -> {
                // Update current offset
                Translation2d offset = calculateOffsetToRobotCenter();

                _calculatedOffsetToRobotCenter = _calculatedOffsetToRobotCenter
                    .times((double) _calculatedOffsetToRobotCenterCount
                        / (_calculatedOffsetToRobotCenterCount + 1))
                            .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
                _calculatedOffsetToRobotCenterCount++;

                SmartDashboard.putNumberArray("Quest Calculated Offset to Robot Center", new double[] {
                    _calculatedOffsetToRobotCenter.getX(), _calculatedOffsetToRobotCenter.getY() });

            }).onlyIf(() -> getRotation().getMeasureZ().in(Degrees) > 30 || getRotation().getMeasureZ().in(Degrees) < -90))
            .onlyWhile(() -> getRotation().getMeasureZ().in(Degrees) >= 0 || getRotation().getMeasureZ().in(Degrees) < -90);
    }
}