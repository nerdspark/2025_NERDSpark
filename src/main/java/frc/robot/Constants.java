// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;


import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CoralConstants {
    public static final double indexerCurrentLimit = 30;
    public static final String canBus = "canivore1";
    public static final double deployCurrentLimit = 65;
    public static final double deployRampRate = 0.03;
    public static final double deployOffset = 0.25-0.1; 
    public static final double homePositionIntake = deployOffset + 0.07; // deployOffset + 0.02
    public static final double deployPositionIntake = deployOffset + 0.325; // 0.27 for algae
    public static final double transferPositionIntake = deployPositionIntake - 0.15; 
    // public static final double elevatorPositionIntake = homePositionIntake + 0.1;
    public static final double forwardLimitDeploy = deployPositionIntake;
    public static final double reverseLimitDeploy = homePositionIntake;
    public static final double kPDeploy = 12; // 12 for 25:1; 25 for 75 : 1
    public static final double kIDeploy = 0;
    public static final double kDDeploy = 0;
    public static final double kGDeploy = 0.42; // 0.39 for 25:1; 0.07 for 75:1
    public static final double intakeCurrentLimit = 65;
    public static final double deployGearRatio = 25.0;
    public static final double deploySensorRatio = deployGearRatio;
    public static final double intakeGearRatio = 5.0;
    public static final double indexerGearRatio = 20.0;
    public static final int shooterID = 1;
    public static final int indexerID = 2;
    public static final int elevatorLeftID = 3;
    public static final int elevatorRightID = 4;
    public static final int deployID = 5;
    public static final int intakeID = 6;
    public static final int intakeSensorID = 7;
    public static final int indexerSensorID = 8;
    public static final double intakeSensorTriggerDistance = 0.09;
    public static final double indexerSensorTriggerDistance = 0.02;
    public static final double indexerTransferVoltage = 16;
    public static final double intakeTransferVoltage = 16;
    public static final double intakingVoltage = 16;
    // public static final double shooterTransferVoltage = 2;
    public static final double shooterRewindVoltage = -1.5;
    // public static final double elevatorTransferPosition = 2.9;
    public static final double deployTolerance = 0.06;
    public static enum coralState {
      empty, 
      coralInRange,
      coralInIntake, 
      coralInIndexer
    }
    public static enum elevatorLevel {
      home(0, 2, 0), 
      l1(1,13.85, 1.45),
      l1upper(1,l1.height + 5, 1.45),
      l1inside(1, l1.height + 5, 2.5),
      l2(2,22.0, 3.5), 
      transfer(0, 3.6, 2),
      panic(0, 7, 1), 
      visionClear(0, 5, 0);
      
      elevatorLevel(int level, double height, double shootVoltage) {
        this.height = height;
        this.level = level; 
        this.shootVoltage = shootVoltage;
      }
      public static elevatorLevel fromLevel(int level) {
        for (elevatorLevel elev : elevatorLevel.values()) {
          if (elev.level == level) {
            return elev;
          }
        }
        return home;
      }
      public final double shootVoltage;
      public final double height;
      public final int level;
  
    }

    
    public static final int pulleyTeeth = 12;
    public static final double pulleyToothWidth = 5.0; // mm
    public static final double pulleyCircumferenceMillimeters = pulleyTeeth * pulleyToothWidth; // mm
    public static final double pulleyCircumferenceInches = pulleyCircumferenceMillimeters / 25.4; // inches
    public static final double elevatorSensorRatio = 1/pulleyCircumferenceInches; // rot/inch
    public static final double kP = 0.70; //1.05
    public static final double kI = 0.00;
    public static final double kD = 0.0;
    public static final double kG = 0.42; //0.385
    public static final double kS = 0.0; 
    public static final double elevatorCurrentLimit = 50;
    public static final double elevatorRampRate = 0.05;
    public static final double elevatorTolerance = 0.15; // in
    public static final double homePos = 1.5; // in
    public static final double forwardLimit = 24;
    public static final double reverseLimit = 0;

    public static final double shooterCurrentLimit = 40;

    
  }
  
  public static class ClimbConstants {
    public static final double gearRatio = 125.0;
    public static final double pulleyDiameter = 2.5; // inches
    public static final double inchesPerRotation = pulleyDiameter * 2.0 * Math.PI / gearRatio;
    public static final int winchId = 61;
    public static final double currentLimit = 20;
    public static final double power = 0.7;
    public static final double deployPositionInches = -10.5;
    public static final double climbedPositionInches = 32;
    public static final double deployPosition = deployPositionInches / inchesPerRotation; // rot of kraken
    public static final double homePosition = -0; // rot of kraken
    public static final double climbedPosition = climbedPositionInches / inchesPerRotation; // rot of kraken
    public static final double rampRate = 0.08;
    public static final double kP = 1.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final String canBus = "canivore1";
    // public static final double servoOpenPosition = 1.0;
    // public static final double servoCloseposition = 0.0;
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
    joystickMap.put(0.0, 0.0);
    joystickMap.put(0.1, 0.02);
    joystickMap.put(0.2, 0.05);
    joystickMap.put(0.3, 0.1);
    joystickMap.put(0.4, 0.15);
    joystickMap.put(0.5, 0.21);
    joystickMap.put(0.6, 0.30);
    joystickMap.put(0.7, 0.45);
    joystickMap.put(0.8, 0.62);
    joystickMap.put(0.9, 0.85);
    joystickMap.put(0.95, 1.00);
    joystickMap.put(1.0, 1.00);

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

public static class AutoDropoff {
  public static final double robotThickness = Units.inchesToMeters(11+3.125);
  public static final double distanceToAutoDrive = Units.feetToMeters(4); // distance between station and bumpers
  public static final double L1waitToHome = 1.5; // s
  public static final double loopPeriodSecs = 0.02;
  public static final ProfiledPIDController driveController =
      new ProfiledPIDController(
          15, 0, 0.1, new TrapezoidProfile.Constraints(Constants.Vision.MAX_VELOCITY,Constants.Vision.MAX_ACCELARATION), loopPeriodSecs); //10, 0, 0
  public static final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          7, 0,0, new TrapezoidProfile.Constraints(Math.toRadians(Constants.Vision.MAX_VELOCITY_ROTATION), Math.toRadians(Constants.Vision.MAX_ACCELARATION_ROTATION)), loopPeriodSecs); //3, 10, 0

}
public static class Vision {


        public static boolean DOGLOG_ENABLED = false;

        public static final boolean USE_VISION = true;
        public static final boolean USE_QUESTNAV = true;

        public static final boolean USE_BUTTON_BOARD = true;



        public static final String kCameraNameFront = "LeftCamera";
        public static final Transform3d kRobotToCamFront =
                new Transform3d(new Translation3d(-Units.inchesToMeters(8.503), Units.inchesToMeters((6.281)), Units.inchesToMeters(8.216)), 
                new Rotation3d(Math.toRadians(-0), Math.toRadians(-15), Math.toRadians(180+35))); //TODO: determine XYZ


        public static final String kCameraNameBack = "RightCamera";
        public static final Transform3d kRobotToCamBack =
                new Transform3d(new Translation3d(-Units.inchesToMeters(8.503), -Units.inchesToMeters((6.281)), Units.inchesToMeters(8.216)), 
                new Rotation3d(Math.toRadians(-0), Math.toRadians(-15), Math.toRadians(180-35))); //TODO: determine XYZ

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        //Do not change these. Actual values will be calculated by the vision system.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
        //Change these for fine tune vision system calculations of standard deviations.
        public static final double kXYStdDev = 0.4; 
        public static final double kThetaStdDev = 1; 

        public static final double TRANSLATION_TOLERANCE_X = 0.013; // Changed from 0.05 3/8/25
        public static final double TRANSLATION_TOLERANCE_Y = 0.013; // Changed from 0.05 3/8/25
        public static final double ROTATION_TOLERANCE = Math.toRadians(1.3); // /deg

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
        // public static final double kLimeLightHeight = 0.120;
        // public static final double kLimeLightXOffset = 0;
        // public static final double kLimeLightYOffset = 0;
        // public static final double kLimeLightAOD = -15.0;

        //NERDSwerve Dims.
        // public static final double kLimeLightHeight = 0.18;
        // public static final double kLimeLightXOffset = 0;
        // public static final double kLimeLightYOffset = 0.14;
        // public static final double kLimeLightAOD = -15.0;

        //Comp. Dims.
        public static final double kLimeLightHeight = 1.02997;
        public static final double kLimeLightXOffset = 0;
        public static final double kLimeLightYOffset = -0.0007366;
        public static final double kLimeLightAOD = -40.0;

        public static boolean kCoralTargeted = false;
        public static boolean kCoralInRange = false;
        // public static boolean kCoralAutoTarget = true;

        public static final boolean USE_LIMELIGHT = true;
        


        public static final Map<ReefLevel, Transform2d> reefLevelOffsetsMap = new HashMap<>();
        static {

          // reefLevelOffsetsMap.put(ReefLevel.L1Inside, new Transform2d(Units.inchesToMeters(24), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L1Top, new Transform2d(Units.inchesToMeters(26), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L1, new Transform2d(Units.inchesToMeters(26), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L2, new Transform2d(Units.inchesToMeters(23), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L3, new Transform2d(Units.inchesToMeters(28.5), 0, new Rotation2d(Math.toRadians(0))));
          reefLevelOffsetsMap.put(ReefLevel.L4, new Transform2d(Units.inchesToMeters(25.5), 0, new Rotation2d(Math.toRadians(0))));
          
          
        }
        public static final Transform2d algaeOffset = new Transform2d(Units.inchesToMeters(20), 0, new Rotation2d(Math.PI));

        public static final Map<CoralStations, Transform2d> coralStationOffSetsMap = new HashMap<>();
        static {
          coralStationOffSetsMap.put(CoralStations.LEFT, new Transform2d(Units.inchesToMeters(16.5), 0, new Rotation2d(Math.toRadians(180))));
          coralStationOffSetsMap.put(CoralStations.RIGHT, new Transform2d(Units.inchesToMeters(16.5), 0, new Rotation2d(Math.toRadians(180))));
         
        }
        
        public static final Set<Integer> nonReefTagFiducialIDs = new HashSet<>(Set.of(1, 2, 3, 4, 5, 12, 13, 14, 15, 16));

        public static final boolean QUEST_ENABLED = false;

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
  
  
  

public static class LEDConstants {
  public static final double scrollSpeed = 40; 
  public static final double numOfSteps = 3.0;
  public static final int kPort = 0;
  public static final int kLength = 125;
  public static final double blinkSeconds = 1.0;
  public static InterpolatingDoubleTreeMap driveToPoseDistanceMap = new InterpolatingDoubleTreeMap();
    static {
      // Key: cardinal joystick distance
      // Value: % max speed
      driveToPoseDistanceMap.put(0.02, 0.00);
      driveToPoseDistanceMap.put(0.021, 0.25);
      driveToPoseDistanceMap.put(0.05, 0.35);
      driveToPoseDistanceMap.put(0.1, 0.45);
      driveToPoseDistanceMap.put(0.2, 0.55);
      driveToPoseDistanceMap.put(0.5, 0.65);
      driveToPoseDistanceMap.put(1.0, 0.80);
      driveToPoseDistanceMap.put(3.0, 1.0);
    }
}

}
