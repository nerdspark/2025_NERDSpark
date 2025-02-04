package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.PubSubOption;

public class QuestNav {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private PubSubOption periodic = PubSubOption.periodic(0.1);
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
  private DoubleArrayPublisher questResetPose = nt4Table.getDoubleArrayTopic("resetpose").publish(periodic);

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f}, periodic);
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f}, periodic);
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();

  // Gets the Quest's measured position.
  public Pose2d getPose() {
    return new Pose2d(getQuestNavPose().minus(resetPosition).getTranslation(), Rotation2d.fromDegrees(getQuestYaw180()));
  }

  // Gets the battery percent of the Quest.
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  public boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp.
  public double timestamp() {
    return questTimestamp.get();
  }

  // Zero the relativerobot heading
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public void zeroPosition() {
    resetPosition = getPose();
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Get the yaw Euler angle of the headset in 180 to -180
  private float getQuestYaw180() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1];
    //ret %= 360;
    //if (ret < 0) {
      //ret += 360;
    //}
    SmartDashboard.putNumber("Ret Before Change", ret);
    if (ret > 180) {
      SmartDashboard.putBoolean("Ret > 180?", true);
      ret += -360;
    } else {
      SmartDashboard.putBoolean("Ret > 180?", false);
    }
    SmartDashboard.putNumber("Ret After Change", -ret);
    //if (ret < -180) {
      //ret += 360;
    //}
    return -ret;
  }

  private Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    return new Translation2d(-questnavPosition[0], -questnavPosition[2]); // Switch Around Based On Which Way Quest Is Facing/Mounted
  }

  private Pose2d getQuestNavPose() {
    var oculousPositionCompensated = getQuestNavTranslation().minus(new Translation2d(-0.4, -1.6351)); // 6.5 or 0.1651. This is used for starting pose since quest starts at 0
    var angles = questEulerAngles.get();
    var rot2d = Rotation2d.fromDegrees(-angles[1]);
    SmartDashboard.putNumber("rot2d", rot2d.getDegrees());
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getQuestYaw180())); // Rotation2d.fromDegrees(-angles[1]) Works but not in AdvantageScope
  }

  private Field2d field = new Field2d(); 
  
  public void getQuestNavFieldPose() {
    getQuestYaw180();
    field.setRobotPose(getQuestNavPose());
    SmartDashboard.putData("Robot Pose in Field From QuestNav", field);
    
  }

  public Pose2d getPPQuestPose() {
    var questPoseComp = getQuestNavTranslation().minus(new Translation2d());
    return new Pose2d(questPoseComp, Rotation2d.fromDegrees(getQuestYaw180()));
  }
  
  public void PPZeroPose(Pose2d pose) {
    if (questMiso.get() != 98) {
      questResetPose.set(ToArray(pose));
      questMosi.set(2);
    }
  }

  private static double[] ToArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
  }
}