// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

public static class Vision {

        public static final boolean USE_VISION = true;

        public static final String kCameraNameFront = "FrontCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCamFront =
                new Transform3d(new Translation3d(0, 0.0, 0.23495), new Rotation3d(0, 0, Math.toRadians(0)));


        public static final String kCameraNameBack = "BackCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCamBack =
                new Transform3d(new Translation3d(-0.1778, 0.0, 0.23495), new Rotation3d(0, 0, Math.toRadians(180)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

          

        //Do not change these. Actual values will be calculated by the vision system.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
        //Change these for fine tune vision system calculations of standard deviations.
        public static final double kXYStdDev = 0.4; 
        public static final double kThetaStdDev = 1; 

        public static final double TRANSLATION_TOLERANCE_X = 0.05; // Changed from 0.05 3/26/23
        public static final double TRANSLATION_TOLERANCE_Y = 0.05; // Changed from 0.05 3/26/23
        public static final double ROTATION_TOLERANCE = 5.0; // /deg

        public static final double MAX_VELOCITY = 4; // 3 //2
        public static final double MAX_ACCELARATION = 10; // 2 //1
        public static final double MAX_VELOCITY_ROTATION = 16; // 8
        public static final double MAX_ACCELARATION_ROTATION = 16; // 8
        
        public static final double VELOCITY_TOLERANCE_X =5;
        public static final double VELOCITY_TOLERANCE_Y = 5;
        public static final double VELOCITY_TOLERANCE_OMEGA = 5;


        public static final double kPXController = 2.5d;
        public static final double kIXController = 0.1d;
        public static final double kDXController = 0d;
        public static final double kPYController = 2.5d;
        public static final double kIYController = 1d; //0.1d;
        public static final double kDYController = 0d;
        public static final double kIzoneX = 1.0d;
        public static final double kIzoneY = 1.0d;
        public static final double kPThetaController = 2;
        public static final double kIThetaController = 0;
        public static final double kDThetaController = 0.041; //0.0041
        public static final double IZone = 5;
        public static final double autoTurnCeiling = 5.0;


        public static final double  kPoseAmbiguityThreshold = 0.2;
        public static final double  kSingleTagDistanceThreshold =2.0;

        
  //      public static HashMap<String, Pose2d> reefPositions = new HashMap<>();

//         static {        

//           reefPositions.put("A0", FieldConstants.Reef.branchPositions.get(0).get(FieldConstants.ReefHeight.L1).toPose2d().plus(new Transform2d(0,0, new Rotation2d(Math.toRadians(180)))));

// //          reefPositions.put("A0", new Pose2d(2, 2, new Rotation2d()));
//           reefPositions.put("A1", new Pose2d());
//           reefPositions.put("A2", new Pose2d());
//           reefPositions.put("A3", new Pose2d());
//           reefPositions.put("A4", new Pose2d());

//           reefPositions.put("B0", FieldConstants.Reef.branchPositions.get(4).get(FieldConstants.ReefHeight.L1).toPose2d().plus(new Transform2d(0,0, new Rotation2d(Math.toRadians(180)))));
//           reefPositions.put("B1", new Pose2d());
//           reefPositions.put("B2", new Pose2d());
//           reefPositions.put("B3", new Pose2d());
//           reefPositions.put("B4", new Pose2d());

//           reefPositions.put("C0", new Pose2d(7, 2, new Rotation2d(Math.toRadians(270))));
//           reefPositions.put("C1", new Pose2d());
//           reefPositions.put("C2", new Pose2d());
//           reefPositions.put("C3", new Pose2d());
//           reefPositions.put("C4", new Pose2d());

//           reefPositions.put("D0", new Pose2d());
//           reefPositions.put("D1", new Pose2d());
//           reefPositions.put("D2", new Pose2d());
//           reefPositions.put("D3", new Pose2d());
//           reefPositions.put("D4", new Pose2d());

//           reefPositions.put("E0", new Pose2d());
//           reefPositions.put("E1", new Pose2d());
//           reefPositions.put("E2", new Pose2d());
//           reefPositions.put("E3", new Pose2d());
//           reefPositions.put("E4", new Pose2d());

//           reefPositions.put("F0", new Pose2d());
//           reefPositions.put("F1", new Pose2d());
//           reefPositions.put("F2", new Pose2d());
//           reefPositions.put("F3", new Pose2d());
//           reefPositions.put("F4", new Pose2d());

//           reefPositions.put("G0", new Pose2d());
//           reefPositions.put("G1", new Pose2d());
//           reefPositions.put("G2", new Pose2d());
//           reefPositions.put("G3", new Pose2d());
//           reefPositions.put("G4", new Pose2d());

//           reefPositions.put("H0", new Pose2d());
//           reefPositions.put("H1", new Pose2d());
//           reefPositions.put("H2", new Pose2d());
//           reefPositions.put("H3", new Pose2d());
//           reefPositions.put("H4", new Pose2d());

//           reefPositions.put("I0", new Pose2d());
//           reefPositions.put("I1", new Pose2d());
//           reefPositions.put("I2", new Pose2d());
//           reefPositions.put("I3", new Pose2d());
//           reefPositions.put("I4", new Pose2d());

//           reefPositions.put("J0", new Pose2d());
//           reefPositions.put("J1", new Pose2d());
//           reefPositions.put("J2", new Pose2d());
//           reefPositions.put("J3", new Pose2d());
//           reefPositions.put("J4", new Pose2d());

//           reefPositions.put("K0", new Pose2d());
//           reefPositions.put("K1", new Pose2d());
//           reefPositions.put("K2", new Pose2d());
//           reefPositions.put("K3", new Pose2d());
//           reefPositions.put("K4", new Pose2d());

//           reefPositions.put("L0", new Pose2d());
//           reefPositions.put("L1", new Pose2d());
//           reefPositions.put("L2", new Pose2d());
//           reefPositions.put("L3", new Pose2d());
//           reefPositions.put("L4", new Pose2d());
        

//         for (int i = 0; i < FieldConstants.Reef.branchPositions.size(); i++) {
//           for (FieldConstants.ReefHeight height : FieldConstants.ReefHeight.values()) {
//             DogLog.log("Target Pose "+ i + " " + height.toString(), FieldConstants.Reef.branchPositions.get(i).get(height).toPose2d());
          
//         }
//       }
//     }
  }
        
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

    
