package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.networktables.DoubleArrayPublisher;
//import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
//import edu.wpi.first.networktables.IntegerPublisher;
//import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NerdQuestNav {
    // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
    NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    NetworkTable nt4Table = nt4Instance.getTable("questnav");   
    private PubSubOption periodic = PubSubOption.periodic(0);
    //private PubSubOption Speriodic = PubSubOption.periodic(60);
    //private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0, periodic);
    //private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish(periodic);
    //private DoubleArrayPublisher questResetPose = nt4Table.getDoubleArrayTopic("resetpose").publish(periodic);

    // Subscribe to the Network Tables questnav data topics
    //private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f, periodic);
    private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f}, periodic);
    //private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f}, periodic);
    //private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f, Speriodic);

    // Position of the quest on the robot (13.5" forward, centered side-to-side, pointed forward))
    private final Transform2d robotToQuest = new Transform2d(inchesToMeters(-11.375), 0.0, Rotation2d.fromDegrees(0));

    // Pose of the robot when the pose was reset
    private Pose2d resetPoseRobot = new Pose2d();

    // Pose of the Quest when the pose was reset
    private Pose2d resetPoseOculus = new Pose2d();

    private Field2d fieldQ = new Field2d();

    /**
    * Gets the pose of the robot on the field
    * 
    * @return pose of the robot
    */
    public Pose2d getRobotPose() {
        fieldQ.setRobotPose(getQuestPose().transformBy(robotToQuest.inverse()));
        SmartDashboard.putData("QuestNav", fieldQ);
        return getQuestPose().transformBy(robotToQuest.inverse());
    }

    /**
    * Gets the pose of the Quest on the field
    * 
    * @return pose of the Quest
    */
    public Pose2d getQuestPose() {
        var rawPose = getUncorrectedOculusPose();
        var poseRelativeToReset = rawPose.minus(resetPoseOculus);
        return resetPoseRobot.transformBy(poseRelativeToReset);
    }

    /**
    * Gets the raw pose of the oculus, relative to the position where it started
    * 
    * @return pose of the oculus
    */
    private Pose2d getUncorrectedOculusPose() {
        var eulerAngles = questEulerAngles.get();
        var rotation = Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

        var questnavPosition = questPosition.get();
        var translation = new Translation2d(-questnavPosition[2], questnavPosition[0]);
        return new Pose2d(translation, rotation);
    }

    /**
    * Set the robot's pose on the field. This is useful to seed the robot to a known position. This is usually called at
    * the start of the autonomous period.
    * 
    * @param newPose new robot pose
    */
    public void resetPose(Pose2d newPose) {
        resetPoseOculus = getUncorrectedOculusPose().transformBy(robotToQuest.inverse());
        resetPoseRobot = newPose;
    }
}