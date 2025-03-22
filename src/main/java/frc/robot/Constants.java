// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants.CoralStations;
import frc.robot.FieldConstants.ReefLevel;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.math.util.Units;
import java.lang.annotation.Documented;
import java.lang.reflect.Array;
import java.rmi.MarshalException;
import java.text.CollationElementIterator;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ArmPath;
import frc.robot.util.ArmPoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class ClimbConstants {
    public static final int kLeftID = 61;
    public static final int kRightID = 62;
    public static final double currentLimit = 105;
    public static final double ampTriggeredCurrentLimit = 25;
    public static final double power = -0.16;
    public static final double deployPosition = 76; // rot
    public static final double climbedPosition = 22; // rot
    public static final double rampRate = 0.2;
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;
  
    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";
  
    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmKp = 50.0;
    public static final double kDefaultArmSetpointDegrees = 75.0;
  
    // distance per pulse = (angle per revolution) / (pulses per revolution)
    //  = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
  
    public static final double kArmReduction = 200;
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kMinAngleRads = Units.degreesToRadians(-75);
    public static final double kMaxAngleRads = Units.degreesToRadians(255);

    
  public static InterpolatingDoubleTreeMap joystickMap = new InterpolatingDoubleTreeMap();
  static {
    // Key: cardinal joystick distance
    // Value: % max speed
    joystickMap.put(0.00, 0.00);
    joystickMap.put(0.07, 0.05);
    joystickMap.put(0.18, 0.1);
    joystickMap.put(0.29, 0.15);
    joystickMap.put(0.40, 0.2);
    joystickMap.put(0.50, 0.3);
    joystickMap.put(0.60, 0.50);
    joystickMap.put(0.70, 0.65);
    joystickMap.put(0.80, 0.80);
    joystickMap.put(0.90, 1.00);
    joystickMap.put(1.00, 1.00);

    joystickMap.put(-0.07, -0.10);
    joystickMap.put(-0.18, -0.15);
    joystickMap.put(-0.29, -0.20);
    joystickMap.put(-0.40, -0.25);
    joystickMap.put(-0.50, -0.35);
    joystickMap.put(-0.60, -0.50);
    joystickMap.put(-0.70, -0.65);
    joystickMap.put(-0.80, -0.80);
    joystickMap.put(-0.90, -1.00);
    joystickMap.put(-1.00, -1.00);
  }
  }


public static class Vision {

        public static boolean DOGLOG_ENABLED = false;

        public static final boolean USE_VISION = true;

        public static final boolean USE_BUTTON_BOARD = true;



        public static final String kCameraNameFront = "LeftCamera";
        public static final Transform3d kRobotToCamFront =
                new Transform3d(new Translation3d(Units.inchesToMeters(14 - 5.03), Units.inchesToMeters((12 - 5.56)), Units.inchesToMeters(17.00)), 
                new Rotation3d(Math.toRadians(-0), Math.toRadians(20), Math.toRadians(15))); //TODO: determine XYZ


        public static final String kCameraNameBack = "RightCamera";
        public static final Transform3d kRobotToCamBack =
                new Transform3d(new Translation3d(Units.inchesToMeters(14 - 5.03), -Units.inchesToMeters((12 - 5.56)), Units.inchesToMeters(17.00)), 
                new Rotation3d(Math.toRadians(-0), Math.toRadians(20), Math.toRadians(-15))); //TODO: determine XYZ

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        //Do not change these. Actual values will be calculated by the vision system.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
        //Change these for fine tune vision system calculations of standard deviations.
        public static final double kXYStdDev = 0.4; 
        public static final double kThetaStdDev = 1; 

        public static final double TRANSLATION_TOLERANCE_X = 0.01; // Changed from 0.05 3/8/25
        public static final double TRANSLATION_TOLERANCE_Y = 0.01; // Changed from 0.05 3/8/25
        public static final double ROTATION_TOLERANCE = Math.toRadians(1.0); // /deg

        //Below same as pathplanner constants
        public static final double MAX_VELOCITY = 4.5; 
        public static final double MAX_ACCELARATION = 2; 
        public static final double MAX_VELOCITY_ROTATION = 540; 
        public static final double MAX_ACCELARATION_ROTATION = 720;
        
        public static final double VELOCITY_TOLERANCE_X = 4;
        public static final double VELOCITY_TOLERANCE_Y = 4;
        public static final double VELOCITY_TOLERANCE_OMEGA = 5;
        public static final double kPXController = 10; //2.5
        public static final double kIXController = 0.01d ; //0.01d
        public static final double kDXController = 0d;
        public static final double kPYController = 10; //2.5
        public static final double kIYController = 0.01d ;//0.01
        public static final double kDYController = 0.0d; //0.01d
        public static final double kIzoneX = 1.0d;
        public static final double kIzoneY = 1.0d;
        public static final double kPThetaController = 0.5; //2
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0; //0.0041
        public static final double IZone = 1.0d;
        // public static final double autoTurnCeiling = 5.0;

        public static final double kPoseAmbiguityThreshold = 0.2;
        public static final double kSingleTagDistanceThreshold = 2.0;

        public static final double kAlgaeCenterHeight = 0.2032; //in meters

        public static final double kCoralCenterUprightHeight = 0.225425; //in meters
        public static final double kCoralCenterFallenHeight = 0.0508; //in meters
        //Testboard Dims.
        


        public static final Map<ReefLevel, Transform2d> reefLevelOffsetsMap = new HashMap<>();
        static {

          // reefLevelOffsetsMap.put(ReefLevel.L1Inside, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L1Top, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(180))));
          reefLevelOffsetsMap.put(ReefLevel.L1, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(180))));
          reefLevelOffsetsMap.put(ReefLevel.L2, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(180))));
          reefLevelOffsetsMap.put(ReefLevel.L3, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(180))));
          reefLevelOffsetsMap.put(ReefLevel.L4, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(180))));
          
          
        }
        public static final Transform2d algaeOffset = new Transform2d(Units.inchesToMeters(20), 0, new Rotation2d(Math.PI));

        public static final Map<CoralStations, Transform2d> coralStationOffSetsMap = new HashMap<>();
        static {
          coralStationOffSetsMap.put(CoralStations.LEFT, new Transform2d(Units.inchesToMeters(16.5), 0, new Rotation2d(Math.toRadians(180))));
          coralStationOffSetsMap.put(CoralStations.RIGHT, new Transform2d(Units.inchesToMeters(16.5), 0, new Rotation2d(Math.toRadians(180))));
         
        }
    }

    public static final double gyroP = 2;
    public static final double gyroI = 0.0;
    public static final double gyroD = 0.00;

    public static final String pigeonCanBus = "canivore1";


//         for (int i = 0; i < FieldConstants.Reef.branchPositions.size(); i++) {
//           for (FieldConstants.ReefHeight height : FieldConstants.ReefHeight.values()) {
//             DogLog.log("Target Pose "+ i + " " + height.toString(), FieldConstants.Reef.branchPositions.get(i).get(height).toPose2d());
          
//         }
//       }
//     }
  
  

public static class BucketConstants {
  public static final int leftSensorPort = 1;
  public static final int rightSensorPort = 2;

  public static final int timesForBucketToTestPositive = 10; // number of consecutive loops a reading must be within detected coral/algae distance in order to test positive
  public static final double coralDistance = 0.15; // maximum distance of distance sensor readings in order to consider coral to be detected
  // public static final double algaeDistance = 0.05; // maximum distance of distance sensor readings in order to consider algae to be detected

}
  public final class ArmGains {
      public static final double shoulderP = 200.0; //TODO CHANGE SOME OF THIS LATER //52.0
      public static final double shoulderI = 0.0;
      public static final double shoulderD = 40.0;
      public static final double elbowP = 200.0;//20.0
      public static final double elbowI = 0.0;
      public static final double elbowD = 40.0;
      public static final double wristP = 300.0; //20.0
      public static final double wristG = 0.0; //20.0 
      public static final double wristI = 0.0;
      public static final double wristD = 0;
      public static final double wristVelocity = 2;
      public static final double wristAcceleration = 2;
      public static final double gripperP = 28.0; // 10.0
      public static final double gripperI = 0.0;
      public static final double gripperD = 0.0;
      public static final double shoulderS = 0.0;
      public static final double shoulderG = 0.6; // 0.25
      public static final double shoulderV = 0.0;
      public static final double shoulderA = 0.0;
      public static final double elbowS = 0.0;
      public static final double elbowG = 0.6;//0.3
      public static final double elbowV = 0.0;
      public static final double elbowA = 0.0;
  }
  public static class ArmVelocityGains{
    public static final double shoulderP = 10.0; //TODO CHANGE SOME OF THIS LATER //52.0
      public static final double shoulderI = 0.0;
      public static final double shoulderD = 0.0;
      public static final double elbowP = shoulderP*0.4;//20.0
      public static final double elbowI = 0.0;
      public static final double elbowD = 0.0;
      public static final double shoulderS = 0.0;
      public static final double shoulderG = 0.6; // 0.25
      public static final double shoulderV = 0.0;
      public static final double shoulderA = 0.0;
      public static final double elbowS = 0.0;
      public static final double elbowG = 0.6;//0.3
      public static final double elbowV = 0.0;
      public static final double elbowA = 0.0;
      
    public static final double lookAheadDistance = 15.0;
    public static final double lookAheadDistanceBeforeInflecting = 1; 
    public static final double endDistance = 13.0;
    public static final double linearApproximationTime = 0.2; // seconds
    public static final double velocity = 65;// does not seem to have any effect
    public static final double maxMotorVelocity = 3.0;
    public static final double arcRadius = 1;
    public static final int arcPoints = 10;
    public static final double interpolationDistance = 0.5; // inches
    public static final double interpolationAngle = 0.2; // deg
  }

    
  public static class ArmConstants {
    
    public static final double elbowSlewRate = 4; // accel in rot/s/s
    public static final double shoulderSlewRate = 4;


    public static final int shoulderMotorLeftPort = 41;
    public static final int shoulderMotorRightPort = 42;
    public static final int elbowMotorLeftPort = 43;
    public static final int elbowMotorRightPort = 44;
    public static final int wristMotorPort = 45;
    public static final int gripperMotorPort = 46;
    public static final String armCanBus = "canivore1";

    public static final double currentLimitShoulder = 25.0;
    public static final double currentLimitElbow = 25.0;
    public static final double currentLimitWrist = 35.0; //40.0
    public static final double gripperCurrentLimitDefault = 10.0;
    public static final double gripperPowerDefault = 0.1;


    //shoulder true offset: 34.513 deg below forward horizontal
    //shoulder gearbox: 75:1
    //shoulder stage 0: 36:26
    public static final double shoulderGearRatio = 125.0*36.0/26.0;
    
    // elbow true offset: 122.198 deg above forward horizontal
    // elbow gearbox: 75:1
    //elbow stage 0: 38:26
    // elbow stage 1: 50:50
    public static final double elbowGearRatio = 75.0*38.0/26.0;

    //wrist up/down gearbox: 25:1
    //wrist up/down stage 0: 50:50
    //wrist up/down stage 1: 50:50
    //wrist up/down stage 2: 35:50
    public static final double wristGearRatio = 25.0*50.0/50.0*35.0/50.0;
    public static final double gripperGearRatio = 5.0*5.0 / 3.0;
    public static final double baseStageLength = 23.158;
    public static final double secondStageLength = 25.475;
    public static final double totalStageLength = baseStageLength + secondStageLength;


    public static final double shoulderRadPerRot = shoulderGearRatio; 
    public static final double elbowRadPerRot = elbowGearRatio; 
    public static final double wristRadPerRot = wristGearRatio;
    public static final double gripperRadPerRot = gripperGearRatio;
    public static final double gripperOffset = 0;
    public static final double shoulderOffset = -0.570 / 2.0 / Math.PI; // radians, fwd = 0
    public static final double elbowOffset = 1.932 / 2.0 / Math.PI; // negative of measurement
    public static final double wristOffset = 4.82 / 2.0 / Math.PI; // 

    /** wrist flip belting ratio between elbow and the wrist */
    public static final double wristToElbowRatio = 1.0/(35.0 / 50.0);
    public static final double rightServoOffset = 0.0;
    public static final double leftServoOffset = 0.0;
    // public static final double onCloseServoPosition = 0.3;
    // public static final double onOpenServoPosition = 0;
    // public static final double openServoPosition = 1;
    // public static final double closeServoPosition = 0;

  }

  public static class ArmSetpoints {

    public static final int setPointCount = 12;
    public static final Translation2d home = new Translation2d(10.6, 11.4);//new Translation2d(15.65, Rotation2d.fromDegrees(60)); //safest home and also closest possible distance arm is allowed to get to central joint
    public static final double homeWrist = Units.degreesToRadians(110);
    /**
     * contains a list of endpoints (0, 0) in arm coordinates = (6.4, 22.0) in bumper-relative coordinates
     * @home 0
     * 
     * **Reef**
     * @L1 1
     * @L2 2 (-2.2, 31.1) from bumper @ 55 degrees to horizontal
     * @L3 3 (-2.2, 47.0) from bumper @ 55 degrees to horizontal
     * @L4 4 (-2.0, 71.9) from bumper @ 90 degrees to horizontal
     * @L2.5Algae 5 (-8, 35.8) from bumper @ 0 degrees to horizontal
     * @L3.5Algae 6 (-8, 51.7) from bumper @ 0 degrees to horizontal
     * 
     * **Coral Intake**
     * @grabFromFunnelPreparePosition 7
     * @groundintake 8
     * 
     * **Algae dropoff**
     * @AlgaeBargePrepare 9 
     * @AlgaeBargeThrow 10 
     * 
     * ** Climb **
     * @climbPosition 11
     */
    public static ArmPoint[] armSetPoints = new ArmPoint[ArmSetpoints.setPointCount]; 
    static{
      armSetPoints[0] = new ArmPoint(home, homeWrist);

      // armSetPoints[1] = new ArmPoint(home.rotateBy(Rotation2d.fromDegrees(65)), Units.degreesToRadians(180));
      // armSetPoints[2] = new ArmPoint(home.rotateBy(Rotation2d.fromDegrees(35)), Units.degreesToRadians(150));
      // armSetPoints[3] = new ArmPoint(new Translation2d(-8, 27), Units.degreesToRadians(145));
      // armSetPoints[4] = new ArmPoint(new Translation2d(ArmConstants.totalStageLength, Rotation2d.fromDegrees(90)), Units.degreesToRadians(120));


      double dropoffDistanceFromBumper = -5.0;
      Translation2d gripperOffset = new Translation2d(9.9, -2.0);
      // Translation2d gripperCoralOffset = gripperOffset.plus(new Translation2d(-1.9, 9.0));
      // Translation2d gripperCoralOffsetInverted = gripperCoralOffset.plus(new Translation2d(0.0, -11.875));
      Translation2d gripperAlgaeOffset = gripperOffset.plus(new Translation2d(8.0, 0.0));

      armSetPoints[1] = new ArmPoint(home, Units.degreesToRadians(110));
      armSetPoints[2] = new ArmPoint(new Translation2d(-10.7, 25), Units.degreesToRadians(180 + 105));//new ArmPoint(new Translation2d(-8.6, 20.1), Units.degreesToRadians(180 + 145)).add(new Translation2d(dropoffDistanceFromBumper, 0)).withGripperOffset(gripperCoralOffsetInverted);
      armSetPoints[3] = new ArmPoint(new Translation2d(-10.7, 27.5), Units.degreesToRadians(105));//new ArmPoint(new Translation2d(-12.6, 28.0), Units.degreesToRadians(125)).add(new Translation2d(dropoffDistanceFromBumper, 0)).withGripperOffset(gripperCoralOffset);
      armSetPoints[4] = new ArmPoint(new Translation2d(ArmConstants.totalStageLength, Rotation2d.fromDegrees(97)), Units.degreesToRadians(115));//new ArmPoint(new Translation2d(-8.4, 49.9), Units.degreesToRadians(150)).add(new Translation2d(dropoffDistanceFromBumper, 0)).withGripperOffset(gripperCoralOffset);

      armSetPoints[5] = new ArmPoint(new Translation2d(-14.4, 26.0), Units.degreesToRadians(180+0)).add(new Translation2d(dropoffDistanceFromBumper, 0)).withGripperOffset(gripperAlgaeOffset);
      armSetPoints[6] = new ArmPoint(new Translation2d(-14.4, 40.7), Units.degreesToRadians(180+0)).add(new Translation2d(dropoffDistanceFromBumper, 0)).withGripperOffset(gripperAlgaeOffset);

      armSetPoints[7] = new ArmPoint(home, Units.degreesToRadians(225)).rotateElbowBy(Rotation2d.fromDegrees(-30));
      armSetPoints[8] = new ArmPoint(new Translation2d(32.2, -14.6), true, 0.0);

      armSetPoints[9] = new ArmPoint(new Translation2d(30, Rotation2d.fromDegrees(60)), Units.degreesToRadians(45));
      armSetPoints[10] = new ArmPoint(new Translation2d(ArmConstants.totalStageLength, Rotation2d.fromDegrees(60)), Units.degreesToRadians(45));

      armSetPoints[11] = new ArmPoint(home.rotateBy(Rotation2d.fromDegrees(115)), Units.degreesToRadians(240));
      
      for (int i = 0; i < armSetPoints.length; i++) {
        if (armSetPoints[i].position.getNorm() > ArmConstants.totalStageLength) {
          armSetPoints[i] = new ArmPoint(new Translation2d(ArmConstants.totalStageLength, armSetPoints[i].position.getAngle()), armSetPoints[i].inBend, armSetPoints[i].wrist);
          SmartDashboard.putBoolean("armpoint limit trip", true);
          System.out.println("armsetpoint trip " + i);
        } else if (armSetPoints[i].position.getNorm() < ArmSetpoints.home.getNorm()) {
          armSetPoints[i] = new ArmPoint(new Translation2d(ArmSetpoints.home.getNorm(), armSetPoints[i].position.getAngle()), armSetPoints[i].inBend, armSetPoints[i].wrist);
          SmartDashboard.putBoolean("armpoint limit trip", true);
          System.out.println("armsetpoint trip " + i);
        }
        System.out.println("Arm Setpoint " + i + " " + armSetPoints[i].position.toString());
      }
    }

    /** list of dunk setpoints for reef dropoff
     * @L1 1
     * @L2 2 
     * @L3 3 
     * @L4 4 
     */
    public static ArmPoint[] armSetPointsDunk = new ArmPoint[5];
    static {
      armSetPointsDunk[0] = armSetPoints[0];
      armSetPointsDunk[1] = armSetPoints[1];
      armSetPointsDunk[2] = armSetPoints[2].add(new Translation2d(-9, new Rotation2d(armSetPoints[2].wrist + (Math.PI*0.25)))).addToWristFlip(Units.degreesToRadians(-45));
      armSetPointsDunk[3] = armSetPoints[3].add(new Translation2d(-13, new Rotation2d(armSetPoints[3].wrist - (Math.PI*0.4)))).addToWristFlip(Units.degreesToRadians(30));
      armSetPointsDunk[4] = armSetPoints[4].addToWristFlip(Units.degreesToRadians(60));
    }




    /**
     * contains a list of intermediate points from first index to second index
     */
    @SuppressWarnings("unchecked")
    public static List<ArmPoint>[][] intermediatePoints = new List[ArmSetpoints.setPointCount][ArmSetpoints.setPointCount]; 
    static{
      // intermediatePoints[0][1] = (List<ArmPoint>) List.of(new ArmPoint(new Translation2d(22.43, 30.52)), new ArmPoint(new Translation2d(47.519, 10.114)));
intermediatePoints[0][8] = (List<ArmPoint>) List.of((new ArmPoint(new Translation2d(34.5, 16.5), true)));
intermediatePoints[4][8] = (List<ArmPoint>) List.of((new ArmPoint(new Translation2d(34.5, 16.5), true)));

// intermediatePoints[6][5] = List.of(armSetPoints[6].withWrist(Units.degreesToRadians(45)));
      // intermediatePoints[1][4] = (List<ArmPoint>) List.of(new ArmPoint(new Translation2d(30.0, 24.0)));

      for (int i = 0; i < ArmSetpoints.setPointCount; i++){
        for (int j = 0; j < i; j++){
          if (intermediatePoints[i][j] == null && intermediatePoints[j][i] != null){
            intermediatePoints[i][j] = new ArrayList<>();
            intermediatePoints[i][j].addAll(intermediatePoints[j][i]);
            Collections.reverse(intermediatePoints[i][j]);
          } else if (intermediatePoints[i][j] != null && intermediatePoints[j][i] == null){
            intermediatePoints[j][i] = new ArrayList<>();
            intermediatePoints[j][i].addAll(intermediatePoints[i][j]);
            Collections.reverse(intermediatePoints[j][i]);
          } else if (intermediatePoints[i][j] == null && intermediatePoints[j][i] == null) {
            intermediatePoints[i][j] = List.of();
            intermediatePoints[j][i] = List.of();
          }
        } 
      }
    }
    

    // /**
    //  * contains a full path between two points
    //  */
    // public static ArmPath[][] armPaths = new ArmPath[ArmSetpoints.setPointCount][ArmSetpoints.setPointCount]; 
    // static {
    //   for (int i = 0; i < ArmSetpoints.setPointCount; i++){
    //     for (int j = 0; j < ArmSetpoints.setPointCount; j++){
    //       if (i != j){
    //         if (intermediatePoints[i][j].size() != 0) {
    //           armPaths[i][j] = new ArmPath(intermediatePoints[i][j], armSetPoints[i], armSetPoints[j]);
    //         } else {
    //           armPaths[i][j] = new ArmPath(armSetPoints[i], armSetPoints[j]);
    //         }
    //         // System.out.println("Created arm path" + i + " " + j);
    //         // System.out.println(armPaths[i][j]);
    //       } else { // set path to just the endpoint
    //         armPaths[i][j] = new ArmPath(List.of(armSetPoints[i]));
    //       }
    //     }
    //   }
    // }
    
  }
  

  public static class LEDConstants {
    public static final double scrollSpeed = 40; 
    public static final double numOfSteps = 3.0;
    public static final int kPort = 3;
    public static final int kLength = 120;
    public static final double blinkSeconds = 1.0;
  }

}
