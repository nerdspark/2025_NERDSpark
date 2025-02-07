// package frc.robot;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Quaternion;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.networktables.DoubleArrayPublisher;
// import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.networktables.FloatArraySubscriber;
// import edu.wpi.first.networktables.IntegerPublisher;
// import edu.wpi.first.networktables.IntegerSubscriber;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.PubSubOption;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class QuestNavOG {
//   // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
//   NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
//   NetworkTable nt4Table = nt4Instance.getTable("questnav");
//   private PubSubOption periodic = PubSubOption.periodic(0.00833);
//   private PubSubOption Speriodic = PubSubOption.periodic(60);
//   private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0, periodic);
//   private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish(periodic);
//   private DoubleArrayPublisher questResetPose = nt4Table.getDoubleArrayTopic("resetpose").publish(periodic);

//   // Subscribe to the Network Tables questnav data topics
//   private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f, periodic);
//   private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f}, periodic);
//   private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
//   private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f}, periodic);
//   private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f, Speriodic);

//   // Local heading helper variables
//   private float yaw_offset = 0.0f;
//   private Pose2d resetPosition = new Pose2d(0, 0, new Rotation2d()); //-0.42545, -1.651
//   private Field2d field = new Field2d();

//   // Gets the Quest's measured position.
//   public Pose2d getPose() {
//     Pose2d pose = new Pose2d(getQuestNavPose().minus(resetPosition).getTranslation(), Rotation2d.fromDegrees(getOculusYaw()));
//     field.setRobotPose(pose);
//     SmartDashboard.putData("QuestNav In Field", field);
//     SmartDashboard.putNumber("Quest Time Diff", (RobotController.getFPGATime() - questPosition.getLastChange()));
//     return new Pose2d(getQuestNavPose().minus(resetPosition).getTranslation(), Rotation2d.fromDegrees(getOculusYaw()));
//   }

//   // Gets the battery percent of the Quest.
//   public double getBatteryPercent() {
//     return questBatteryPercent.get();
//   }

//   // Returns if the Quest is connected.
//   public boolean connected() {
//     return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
//   }

//   // Gets the Quaternion of the Quest.
//   public Quaternion getQuaternion() {
//     float[] qqFloats = questQuaternion.get();
//     return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
//   }

//   // Gets the Quests's timestamp.
//   public double timestamp() {
//     return questTimestamp.get();
//   }

//   // Zero the relativerobot heading
//   public void zeroHeading() {
//     float[] eulerAngles = questEulerAngles.get();
//     yaw_offset = eulerAngles[1];
//   }

//   // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
//   public void zeroPosition() {
//     resetPosition = getPose();
//     if (questMiso.get() != 99) {
//       questMosi.set(1);
//     }
//   }

//   // Clean up questnav subroutine messages after processing on the headset
//   public void cleanUpQuestNavMessages() {
//     if (questMiso.get() != 0) {
//       questMosi.set(0);
//     }
//   }

//   // Initiallize the pose of the headset to some value
//   public void initPose(Pose2d pose) {
//     while (questMiso.get() == 0 && questMiso.get() != 98) {
//       questResetPose.set(toArray(pose));
//       questMosi.set(2);
//     }
//     cleanUpQuestNavMessages();
//   }

//   // Get the yaw Euler angle of the headset
//   private float getOculusYaw() {
//     float[] eulerAngles = questEulerAngles.get();
//     var ret = -eulerAngles[1] - yaw_offset;
//     ret %= 360;
//     if (ret < 0) {
//       ret += 360;
//     }
//     return ret;
//   }

//   private Translation2d getQuestNavTranslation() {
//     float[] questnavPosition = questPosition.get();
//     return new Translation2d(-questnavPosition[0], -questnavPosition[2]);
//   }

//   private Pose2d getQuestNavPose() {
//     var oculousPositionCompensated = getQuestNavTranslation().plus(new Translation2d(0,0)); //0.1651, 0.1143, -0.2794
//     return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
//   }

//   // Pose2d to Double Array for NT
//   private double[] toArray(Pose2d pose) {
//     return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
//   }
// }