package frc.robot;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class NerdQuestNav {
    // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private PubSubOption periodic = PubSubOption.periodic(0.00833);
  private PubSubOption Speriodic = PubSubOption.periodic(60);
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0, periodic);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish(periodic);
  private DoubleArrayPublisher questResetPose = nt4Table.getDoubleArrayTopic("resetpose").publish(periodic);

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f, periodic);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f}, periodic);
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f}, periodic);
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f, Speriodic);
}
