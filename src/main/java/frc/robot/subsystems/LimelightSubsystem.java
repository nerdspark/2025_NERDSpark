// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CoralObject;
import frc.robot.util.CoralArrayManager;
import static frc.robot.Constants.Vision.kLimeLightHeight;


public class LimelightSubsystem extends SubsystemBase {
       private final NetworkTable llTable;
    private static List<CoralObject> corals = new ArrayList<>();
    private static CoralArrayManager coralManager = new CoralArrayManager();
    private static Gyro gyro = new Gyro();
    private CommandSwerveDrivetrain drivetrain;


  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem(CommandSwerveDrivetrain drivetrain) {
             llTable = NetworkTableInstance.getDefault().getTable("limelight");
             this.drivetrain = drivetrain;

  }

  @Override
  public void periodic() {
    
            //Coral pose code 

            // Pose2d coralPose = getCoralPose();

            // if (hasTarget()) {
            //     double hb = getHB();
            //     CoralObject coral = new CoralObject(coralPose, hb, 0);
            //     coral.setCoralDistance(coral.calcDistance(coral.getPose()));
            //     SmartDashboard.putNumber("hb", coral.getHB());
            //     corals.add(coral);
            // }

            // if (corals.size() > 2) {
            //     int size = corals.size() - 2;
            //     Pose2d coralPoseLast = corals.get(size).getPose();
            //     double coralXLast = coralPoseLast.getX();
            //     double coralYLast = coralPoseLast.getY();
            //     SmartDashboard.putNumber("coralXLast", coralXLast);
            //     SmartDashboard.putNumber("coralYLast", coralYLast);
    
            //coral code
            
            if (Constants.Vision.USE_LIMELIGHT) {

            // CoralObject newCoral = newCoral();
            // if (!newCoral.getIgnored() && visionFront.hasTarget()) {
            //     corals.add(newCoral); 
            // }

            

            corals = coralArrayUpdateReturn();
            SmartDashboard.putNumber("size", corals.size());
            // SmartDashboard.putNumber("coralX", corals.get(corals.size() - 1).getPose().getX());
            // SmartDashboard.putNumber("coralY", corals.get(corals.size() - 1).getPose().getY());
            SmartDashboard.putBoolean("tV", hasTarget());              

            // if (corals.size() > 0) {
            //     int size = corals.size();
            //     CoralObject lastCoral = corals.get(size - 1);
            //     Pose2d lastCoralPose = lastCoral.getPose();
            //     SmartDashboard.putNumber("lastCoralX", lastCoralPose.getX());
            //     SmartDashboard.putNumber("lastCoralY", lastCoralPose.getY());
            //     SmartDashboard.putNumber("size", size);
            // }

            // Pose2d coralPose = newCoral.getPose();
            // SmartDashboard.putNumber("coralX", coralPose.getX());
            // SmartDashboard.putNumber("coralY", coralPose.getY());
        }
            //SmartDashboard.putNumber("pigeon", gyro.getGyro().getDegrees());
       
    // This method will be called once per scheduler run
  }
  
    public CoralObject newCoral() {
        Rotation2d yaw = gyro.getGyro();
        //Pose2d pose = new Pose2d(0.0,0.0,yaw);
        Pose2d pose = drivetrain.getState().Pose;
        double poseX = pose.getX();
        double poseY = pose.getY();

        // SmartDashboard.putNumber("poseX", poseX);
        // SmartDashboard.putNumber("poseY", poseY);

        double tx = getTx();
        double ty = getTy();
        double hb = getHB();
        boolean upfall = false;
        boolean ignored = false;
        boolean targeted = false;

        Translation2d offset = new Translation2d();
        //Translation2d offset = new Translation2d(-0.5588, new Rotation2d((yaw.getDegrees() + tx) * Math.PI / 180));

        //DriverStation.getMatchTime();

        double boundingHeight = 0.0;
        double boundingWidth = 0.0;       

        double[] xys = getCoordinates();
        if (xys.length != 0) { //debug
            boundingHeight = xys[5] - xys[3];
            boundingWidth = xys[2] - xys[0];
            SmartDashboard.putNumber("boundingHeight", boundingHeight);
            SmartDashboard.putNumber("boundingWidth", boundingWidth);
        }

        double distance = 0.0;
        double theta = 0.0;
        if (boundingHeight > boundingWidth) {               
            upfall = false;
            ignored = true;
        } else if (boundingHeight <= boundingWidth && boundingHeight != 0.0) {
            distance = (Constants.Vision.kCoralCenterFallenHeight - kLimeLightHeight) / Math.tan((Constants.Vision.kLimeLightAOD+ty) * (Math.PI / 180)) / Math.cos(tx * Math.PI / 180);
            upfall = true;
            ignored = false;
        } else {
            distance = 0.0;
            SmartDashboard.putString("orientation", "");
            ignored = true;
        }
        if (distance > 0.0) {
            Rotation2d coralOrientation = new   Rotation2d(theta);
            Pose2d coralPose = new Pose2d(-distance * Math.sin((yaw.getDegrees()+tx) * (Math.PI / 180)) + Constants.Vision.kLimeLightXOffset + poseX + offset.getX(), -distance * Math.cos((yaw.getDegrees()+tx) * (Math.PI / 180)) + Constants.Vision.kLimeLightYOffset + poseY + offset.getY(), yaw);
            //Pose2d coralPose = new Pose2d(2 + offset.getX(), 2 + offset.getY(), yaw);
            SmartDashboard.putNumber("distance", distance);
            ignored = false;
            CoralObject newCoral = new CoralObject(coralPose, hb, distance, upfall, targeted, ignored);
            return newCoral;
        } else {
            Pose2d zeroed = new Pose2d(0,0, new Rotation2d(0.0));
            ignored = true;
            CoralObject newCoral = new CoralObject(zeroed, hb, distance, upfall, targeted, ignored);
            // Pose2d coralPose = new Pose2d(2 + offset.getX(), 2 + offset.getY(), yaw);
            // SmartDashboard.putNumber("distance", distance);
            // ignored = false;
            // CoralObject newCoral = new CoralObject(coralPose, hb, distance, upfall, targeted, ignored);
            return newCoral;
        }
    }

    public List<CoralObject> coralArrayUpdateReturn() {
        if (!Constants.Vision.kCoralTargeted) {
            CoralObject newCoral = newCoral();
            double hb = getHB();
            double fps = getFPS();
            corals.add(newCoral);
            coralManager.expiryFilter(corals, hb, fps);
            return corals;
        } else {
            return coralManager.selectCoral(corals);
        }
    }

  
     // - LimeLight

     public double getTx() {
        return llTable.getEntry("tx").getDouble(0.0);
      }
    
      public double getTy() {
        return llTable.getEntry("ty").getDouble(0.0);
      }
    
      public double getTa() {
        return llTable.getEntry("ta").getDouble(0.0);
      }
    
      public boolean hasTarget() {
        return llTable.getEntry("tv").getDouble(0.0) == 1.0;
      }
    
      public long getID() {
        return llTable.getEntry("tid").getInteger(0);
      }
    
      public double[] getRelBotPose() {
        NetworkTableEntry relbotpose = llTable.getEntry("targetpose_cameraspace");
        return relbotpose.getDoubleArray(new double[6]);
      }

      public double[] getBotPose() {
        NetworkTableEntry botpose = llTable.getEntry("botpose");
        return botpose.getDoubleArray(new double[6]);
      }
    
      public void setPipelineNumber(int i) {
        llTable.getEntry("pipeline").setNumber(i);
      }

      public String getObjectClass() {
        return llTable.getEntry("tclass").getString("none");
      }

      public double getHB() {
        return llTable.getEntry("hb").getDouble(0.0);
      }
      
      public double[] getCoordinates() {
        double coords[] = new double[8];
        if (llTable.getEntry("tcornxy").getDoubleArray(new double[1]).length == 8) {
          coords = llTable.getEntry("tcornxy").getDoubleArray(new double[1]);
        }
        return coords;
      }
      
      public double getFPS() {
        double[] hw = llTable.getEntry("hw").getDoubleArray(new double[5]);
        return hw[0];
      }
      
      public double getYaw() {
        double[] botpose = getBotPose();
        double yaw = 0.0;
        if (botpose.length == 6) {
            yaw = botpose[5];
        } else {
            yaw = 0.0;
        }
        return yaw;
      }

}
