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
  public static boolean DOGLOG_ENABLED = true;

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
    joystickMap.put(0.07, 0.10);
    joystickMap.put(0.18, 0.15);
    joystickMap.put(0.29, 0.20);
    joystickMap.put(0.40, 0.25);
    joystickMap.put(0.50, 0.35);
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

        public static final boolean USE_VISION = true;

        public static final boolean USE_BUTTON_BOARD = true;



        public static final String kCameraNameFront = "LeftCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCamFront =
                new Transform3d(new Translation3d(-Units.inchesToMeters(12), -Units.inchesToMeters(2), Units.inchesToMeters(13.5)), new Rotation3d(0
                , Math.toRadians(0), Math.toRadians(-157.5))); //0


        public static final String kCameraNameBack = "RightCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCamBack =
                new Transform3d(new Translation3d(-Units.inchesToMeters(12), Units.inchesToMeters(2), Units.inchesToMeters(13.5)), new Rotation3d(0, Math.toRadians(0), Math.toRadians(157.5))); //180

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);          

        //Do not change these. Actual values will be calculated by the vision system.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
        //Change these for fine tune vision system calculations of standard deviations.
        public static final double kXYStdDev = 0.4; 
        public static final double kThetaStdDev = 1; 

        public static final double TRANSLATION_TOLERANCE_X = 0.05; // Changed from 0.05 3/26/23
        public static final double TRANSLATION_TOLERANCE_Y = 0.05; // Changed from 0.05 3/26/23
        public static final double ROTATION_TOLERANCE = 5.0; // /deg

        //Below same as pathplanner constants
        public static final double MAX_VELOCITY = 4.5; 
        public static final double MAX_ACCELARATION = 6; 
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
        // public static final double kLimeLightHeight = 0.120;
        // public static final double kLimeLightXOffset = 0;
        // public static final double kLimeLightYOffset = 0;
        // public static final double kLimeLightAOD = -15.0;
        //Comp. Dims.
        public static final double kLimeLightHeight = 0.56552592;
        public static final double kLimeLightXOffset = -0.17145;
        public static final double kLimeLightYOffset = -0.18415;
        public static final double kLimeLightAOD = -15.0;

        public static boolean kCoralTargeted = false;

        


        public static final Map<ReefLevel, Transform2d> reefLevelOffsetsMap = new HashMap<>();
        static {

          reefLevelOffsetsMap.put(ReefLevel.L0, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L1, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L2, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L3, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L4, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L5, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(180))));
        }
    



        

        public static final Map<CoralStations, Transform2d> coralStationOffSetsMap = new HashMap<>();
        static {
          coralStationOffSetsMap.put(CoralStations.LEFT, new Transform2d(Units.inchesToMeters(6), 0, new Rotation2d(Math.toRadians(180))));
          coralStationOffSetsMap.put(CoralStations.RIGHT, new Transform2d(Units.inchesToMeters(6), 0, new Rotation2d(Math.toRadians(180))));
         
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
  
  

  public final class ArmGains {
      public static final double shoulderP = 80.0; //TODO CHANGE SOME OF THIS LATER //52.0
      public static final double shoulderI = 0.0;
      public static final double shoulderD = 0.0;
      public static final double elbowP = 45.0;//20.0
      public static final double elbowI = 0.0;
      public static final double elbowD = 0.0;
      public static final double wristFlipP = 25.0; //20.0
      public static final double wristFlipG = 1.0; //20.0
      public static final double wristFlipI = 0.0;
      public static final double wristFlipD = 0.0;
      public static final double wristTwistP = 15.0; //15.0
      public static final double wristTwistI = 0.0;
      public static final double wristTwistD = 0.0;
      public static final double gripperP = 0.0; // 10.0
      public static final double gripperI = 0.0;
      public static final double gripperD = 0.0;
      public static final double shoulderS = 0.0;
      public static final double shoulderG = 0.25; // 0.25
      public static final double shoulderV = 0.0;
      public static final double shoulderA = 0.0;
      public static final double elbowS = 0.0;
      public static final double elbowG = 0.3;//0.3
      public static final double elbowV = 0.0;
      public static final double elbowA = 0.0;
  }
  public static class ArmVelocityGains{
    public static final double shoulderP = 52.0; //TODO CHANGE SOME OF THIS LATER //52.0
      public static final double shoulderI = 0.0;
      public static final double shoulderD = 0.0;
      public static final double elbowP = 20.0;//20.0
      public static final double elbowI = 0.0;
      public static final double elbowD = 0.0;
      public static final double shoulderS = 0.0;
      public static final double shoulderG = 0.25; // 0.25
      public static final double shoulderV = 0.0;
      public static final double shoulderA = 0.0;
      public static final double elbowS = 0.0;
      public static final double elbowG = 0.3;//0.3
      public static final double elbowV = 0.0;
      public static final double elbowA = 0.0;
  }

    
  public static class ArmConstants {

    public static final double lookAheadDistance = 10.0;
    public static final double endDistance = 10.0;
    public static final double linearApproximationTime = 0.1; // seconds
    public static final double velocity = 25;
    public static final double maxMotorVelocity = 0.5;
    public static final double arcRadius = 1;
    public static final int arcPoints = 10;
    public static final double interpolationDistance = 0.1; // inches
    public static final double interpolationAngle = 1; // deg

    public static final int shoulderMotorLeftPort = 41;
    public static final int shoulderMotorRightPort = 42;
    public static final int elbowMotorLeftPort = 43;
    public static final int elbowMotorRightPort = 44;
    public static final int wristFlipMotorPort = 45;
    public static final int wristTwistMotorPort = 46;
    public static final int gripperMotorPort = 47;
    public static final int rangeMiddlePort = 34;
    public static final int rangeLeftPort = 31;
    public static final int rangeRightPort = 32;
    public static final String armCanBus = "canivore1";

    public static final double shoulderPowerClimb = -1;
    public static final double currentLimitShoulderClimb = 200.0;
    public static final double currentLimitShoulder = 45.0;
    public static final double currentLimitElbow = 45.0;
    public static final double currentLimitWristFlip = 18.0; //40.0
    public static final double currentLimitWristTwist = 15.0;
    public static final double currentLimitGripperOpen = 8.0;
    public static final double currentLimitGripperClose = 45.0;
    public static final double gripperPowerClose = 1.0;
    public static final double gripperPowerOpen = -0.2;

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
    public static final double wristFlipGearRatio = 25.0*50.0/50.0*35.0/50.0;
    //wrist twist gearbox: 25:1
    //wrist twist stage 0: 50:50
    //wrist twist stage 1: 50:50
    //wrist twist stage 2: 35:50
    //wrist twist bevel: 1:1
    public static final double wristTwistGearRatio = 25.0*50.0/50.0*35.0/50.0;
    public static final double gripperGearRatio = 5.0*5.0 / 3.0;
    public static final double baseStageLength = 23.158;
    public static final double secondStageLength = 25.475;
    public static final double totalStageLength = baseStageLength + secondStageLength;


    public static final double shoulderRadPerRot = shoulderGearRatio; 
    public static final double elbowRadPerRot = elbowGearRatio; 
    public static final double wristFlipRadPerRot = wristFlipGearRatio;
    public static final double wristTwistRadPerRot = wristTwistGearRatio;
    public static final double gripperRadPerRot = gripperGearRatio;
    public static final double gripperOffset = 0;
    public static final double shoulderOffset = -0.325 / 2.0 / Math.PI; // TODO fidn these, radians, fwd = 0
    public static final double elbowOffset = 2.39 / 2.0 / Math.PI; // TODO find these, negative of measurement
    public static final double wristFlipOffset = (1.0) / 2.0 / Math.PI; // TODO 2.25.2025: retune (should be approx 0.50 / 2PI)
    public static final double wristTwistOffset = 1.0 / 2.0 / Math.PI;

    /** wrist flip belting ratio between elbow and the wrist */
    public static final double wristFlipToElbowRatio = 1.0/(35.0 / 49.0);
    public static final double wristTwistToElbowRatio = 1.0/(35.0 / 49.0);
    public static final double wristTwistToFlipRatio = -1.0 / 1.0;
    public static final double rightServoOffset = 0.0;
    public static final double leftServoOffset = 0.0;
    // public static final double onCloseServoPosition = 0.3;
    // public static final double onOpenServoPosition = 0;
    // public static final double openServoPosition = 1;
    // public static final double closeServoPosition = 0;

  }

  public static class ArmSetpoints {

    public static final int setPointCount = 12;
    public static final Translation2d home = new Translation2d(14.0,18.0); //safest home and also closest possible distance arm is allowed to get to central joint

    /**
     * contains a list of endpoints
     * @L1Standard 0
     * @L1Inner 1
     * @L1Top 2
     * @L2Reef 3
     * @L3Reef 4
     * @L4Reef 5
     * @stow (home except closer to other setpoints for faster movement) 6
     * @home 7
     * @transfer 8
     * @climbPrepare 9
     * @climbClose 10
     * @transferPullOut 11
     */
    public static ArmPoint[] armSetPoints = new ArmPoint[ArmSetpoints.setPointCount]; 
    static{
      armSetPoints[7] = new ArmPoint(home, false, Math.PI * 0.5, 0.0);

      armSetPoints[0] = new ArmPoint(new Translation2d(27.0, 2.0), true, -0.5, 1.45);
      armSetPoints[1] = new ArmPoint(new Translation2d(31.0, 2.0), true, -0.5, 1.45);
      armSetPoints[2] = new ArmPoint(new Translation2d(29.0, 7.0), true, -0.5, 1.45);
      armSetPoints[3] = new ArmPoint(new Translation2d(24.0, 16.25), true, 0.611, 0.0);
      armSetPoints[4] = new ArmPoint(new Translation2d(24.0, 32.0), true, 0.611, 0.0);
      armSetPoints[5] = new ArmPoint(new Translation2d(ArmConstants.totalStageLength, Rotation2d.fromDegrees(100)), 2.5, 0.3);
      
      armSetPoints[6] = new ArmPoint(new Translation2d(19, 18), true, Units.degreesToRadians(-90), 1.45);
      
      // armSetPoints[9] = new ArmPoint(new Translation2d(ArmConstants.totalStageLength-10, Rotation2d.fromDegrees(50)), false);
      // armSetPoints[10] = new ArmPoint(new Translation2d(ArmConstants.totalStageLength-10, Rotation2d.fromDegrees(0)), false);
      armSetPoints[9] = new ArmPoint(new Translation2d(30.0, 36.0), false, Units.degreesToRadians(-90), 0.0);
      armSetPoints[10] = new ArmPoint(new Translation2d(30.0, 18.0), false, Units.degreesToRadians(-90), 0.0);

      
      armSetPoints[8] = new ArmPoint(new Translation2d(home.getNorm(),Rotation2d.fromDegrees(20)), true, -2.9, 1.45); //19, 7
      armSetPoints[11] = new ArmPoint(new Translation2d(16, 18), true, -2.6, 1.45); //19, 7

      //clamp distance of all setpoints (probably unnecessary)
      // for (int i = 0; i < armSetPoints.length; i++) {
      //   if (armSetPoints[i].position.getNorm() > ArmConstants.totalStageLength) {
      //     armSetPoints[i] = new ArmPoint(new Translation2d(ArmConstants.totalStageLength, armSetPoints[i].position.getAngle()),armSetPoints[i].inBend, armSetPoints[i].wristFlip, armSetPoints[i].wristTwist);
      //   }
      // }
    }




    /**
     * contains a list of intermediate points from first index to second index
     */
    @SuppressWarnings("unchecked")
    public static List<ArmPoint>[][] intermediatePoints = new List[ArmSetpoints.setPointCount][ArmSetpoints.setPointCount]; 
    static{
      // intermediatePoints[0][1] = (List<ArmPoint>) List.of(new ArmPoint(new Translation2d(22.43, 30.52)), new ArmPoint(new Translation2d(47.519, 10.114)));
intermediatePoints[7][8] = (List<ArmPoint>) List.of((new ArmPoint(new Translation2d(ArmConstants.totalStageLength, Rotation2d.fromDegrees(90)))));
// intermediatePoints[6][5] = List.of(armSetPoints[6].withWristFlip(Units.degreesToRadians(45)));
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
  

  public static class IntakeConstants {
    public static final int intakeDeployMotorPort = 33;
    public static final int intakeGrabberMotorPort = 51;
    public static final int intakeRangePort = 3;
    public static final double intakeDeployCurrentLimit = 40; //40
    public static final double intakeGrabberCurrentLimit = 10;
    public static final String intakeCANBus = "canivore1";

    public static final double deploykP = 8; //7
    public static final double deploykI = 0;
    public static final double deploykD = 0.1;
    public static final double deploykG = 0.4;
    public static final double grabberkP = 0;
    public static final double grabberkI = 0;
    public static final double grabberkD = 0;
    public static final double grabberkG = 0;

    public static final double deployOffset = 0.46;
    public static final double deployGearRatio = 20.0;
    public static final double grabberOffset = 0;
    public static final double grabberGearRatio = 5.0;

    public static final double setpoint0 = 0d;
    public static final double setpoint30 = 1d/12d;
    public static final double setpoint45 = 0.125;
    public static final double setpoint60 = 1d/6d;
    public static final double setpoint90 = .25;

    public static final double home = 0.45;
    public static final double deploy = -0.06;
    public static final double climb = 0.05;
    ;

    public static final double intakeTransferPosition = 0.25;
    public static final double intakePassive = -0.08;

    public static final double transferPowerRollers = 1.0; //-0.5
    public static final double intakePowerRollers = -1.0;
  }

  public static final double scrollSpeed = 40; 

}
