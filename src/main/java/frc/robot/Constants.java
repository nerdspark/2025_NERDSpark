// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

        public static final boolean USE_VISION = false;

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

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

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

        
    }

}
