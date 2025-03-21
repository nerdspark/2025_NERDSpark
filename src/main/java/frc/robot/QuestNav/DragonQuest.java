package frc.robot.QuestNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class DragonQuest {
    Double m_mountingXOffset;
    Double m_mountingYOffset;
    Double m_mountingZOffset;
    Double m_mountingPitch;  
    Double m_mountingYaw;    
    Double m_mountingRoll;

    NetworkTable m_networktable;
    IntegerPublisher m_questMosi;
    IntegerSubscriber m_questMiso;
    DoubleArrayTopic m_posTopic;
    IntegerTopic m_frameCountTopic;
    DoubleArrayTopic m_rotationTopic;
    DoubleArrayPublisher m_initialPosePublisher;
    DoubleSubscriber m_battery;

    PubSubOption periodic = PubSubOption.periodic(0);
    PubSubOption sPeriodic = PubSubOption.periodic(30);

    double m_prevFrameCount = 0;
    int m_loopCounter = 0;

    boolean m_hasreset = false;
    boolean m_isConnected = false;

    Transform2d m_questTransform = new Transform2d();
    Transform2d m_resetOffset = new Transform2d();

    public DragonQuest(
        Double mountingXOffset, // <I> x offset of Quest from robot center (forward relative to robot)
        Double mountingYOffset, // <I> y offset of Quest from robot center (left relative to robot)
        Double mountingZOffset, // <I> z offset of Quest from robot center (up relative to robot)
        Double mountingPitch,  // <I> - Pitch of Quest
        Double mountingYaw,    // <I> - Yaw of Quest
        Double mountingRoll    // <I> - Roll of Quest
    ) {
        m_mountingXOffset = mountingXOffset;
        m_mountingYOffset = mountingYOffset;
        m_mountingZOffset = mountingZOffset;
        m_mountingPitch = mountingPitch;
        m_mountingYaw = mountingYaw;
        m_mountingRoll = mountingRoll;
        
        m_networktable = NetworkTableInstance.getDefault().getTable("questnav");
        m_questMosi = m_networktable.getIntegerTopic("mosi").publish(periodic);
        m_questMiso = m_networktable.getIntegerTopic("miso").subscribe(0, sPeriodic);
        m_posTopic = m_networktable.getDoubleArrayTopic("position");
        m_frameCountTopic = m_networktable.getIntegerTopic("frameCount");
        m_rotationTopic = m_networktable.getDoubleArrayTopic("euler angles");
        m_initialPosePublisher = m_networktable.getDoubleArrayTopic("resetpose").publish(periodic);
        m_battery = m_networktable.getDoubleTopic("battery").subscribe(0.0, sPeriodic);
    }

    public Pose2d getEstimatedPose() {
        RefreshNT();

        double[] posarray = m_posTopic.getEntry(new double[3], periodic).get();
        double[] rotationarray = m_rotationTopic.getEntry(new double[3], periodic).get();

        double x = posarray[2]; // Meters
        double y = -posarray[0]; // Meters
        // double z = posarray[1]; // Meters
        // double roll = rotationarray[0]; // Degrees
        // double pitch = rotationarray[2]; // Degrees
        double yaw = -rotationarray[1]; // Degrees

        Pose2d questPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        Pose2d robotPose = questPose.plus(m_questTransform);
        return robotPose;
    }

    public Pose2d getEstimatedPose(boolean newQuest) {
        RefreshNT();

        double[] posarray = m_posTopic.getEntry(new double[3], periodic).get();
        double[] rotationarray = m_rotationTopic.getEntry(new double[3], periodic).get();

        double x = posarray[2]; // Meters
        double y = -posarray[0]; // Meters
        // double z = posarray[1]; // Meters
        // double roll = rotationarray[0]; // Degrees
        // double pitch = rotationarray[2]; // Degrees
        double yaw = -rotationarray[1]; // Degrees

        Pose2d questPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        Pose2d robotPose = questPose.plus(m_resetOffset);
        return robotPose;
    }

    private void RefreshNT() {
        m_posTopic = m_networktable.getDoubleArrayTopic("position");
        m_rotationTopic = m_networktable.getDoubleArrayTopic("eulerAngles");
        m_frameCountTopic = m_networktable.getIntegerTopic("frameCount");

        setIsConnected();
    }

    private void setIsConnected() {
        double currentFrameCount = m_frameCountTopic.getEntry(0, periodic).get();
        if (m_loopCounter > 3)
        {
            if (currentFrameCount != m_prevFrameCount)
            {
                m_loopCounter = 0;
                m_isConnected = true;
            }
            else
            {
                m_isConnected = false;
            }
            m_prevFrameCount = currentFrameCount;
        }

        m_loopCounter = (m_loopCounter > 3) ? 0 : m_loopCounter + 1;
    }

    public void setRobotPose(Pose2d pose) {
        if (!m_hasreset)
        {
            Double x = pose.getX() + Units.inchesToMeters(m_mountingXOffset);
            Double y = pose.getY() + Units.inchesToMeters(m_mountingYOffset);
            Double rot = pose.getRotation().getDegrees() + m_mountingYaw;

            m_initialPosePublisher.set(new double[] {x, y, rot});

            m_questTransform = pose.minus(new Pose2d(x, y, new Rotation2d(rot)));

            if (m_questMiso.get() != 99)
            {
                m_questMosi.set(2);
            }
            m_hasreset = true;
        }
    }

    public void setRobotPose(Pose2d pose, boolean hasreset) {
        m_hasreset = hasreset;
        
        if (!m_hasreset)
        {
            Double x = pose.getX() + Units.inchesToMeters(m_mountingXOffset);
            Double y = pose.getY() + Units.inchesToMeters(m_mountingYOffset);
            Double rot = pose.getRotation().getDegrees() + m_mountingYaw;

            m_initialPosePublisher.set(new double[] {x, y, rot});

            m_questTransform = pose.minus(new Pose2d(x, y, new Rotation2d(rot)));

            if (m_questMiso.get() != 99)
            {
                m_questMosi.set(2);
            }
            m_hasreset = true;
        }
    }

    //TODO: Figure out if Quest is to slow to reset over nt. May need to happen natively on robot
}