// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoDropoff;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Vision;
import frc.robot.util.AllianceFlipUtil;

import static frc.robot.Constants.Vision.reefLevelOffsetsMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.node.POJONode;

import dev.doglog.DogLog;


/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    public static final double cageThickness = Units.inchesToMeters(6);
    public static final double middlePoleThickness = Units.inchesToMeters(12);
    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
    public static final double[] bargeGaps = new double[8];
    static {
        bargeGaps[0] = (fieldWidth + farCage.getY() + cageThickness * 0.5) * 0.5;
        bargeGaps[1] = (farCage.getY() + middleCage.getY()) * 0.5;
        bargeGaps[2] = (closeCage.getY() + middleCage.getY()) * 0.5;
        bargeGaps[3] = (closeCage.getY() - cageThickness * 0.5 + middlePoleThickness * 0.5 + fieldLength * 0.5) * 0.5;
        for (int i = 0; i <= 3; i++) {
            bargeGaps[7 - i] = fieldWidth -bargeGaps[i];
        }
    }
  }
  public static double getClosestBargeGap(Supplier<Pose2d> robotPose) {
    double currentY = robotPose.get().getY();
    double minDistance = Double.MAX_VALUE;
    double chosenGapPos = fieldWidth * 0.5;
    for (double gapPos : Barge.bargeGaps) {
        if (Math.abs(currentY - gapPos) < minDistance) {
            minDistance = Math.abs(currentY - gapPos);
            chosenGapPos = gapPos;
        }
    }
    SmartDashboard.putNumber("chosengappos", chosenGapPos);
    return chosenGapPos;
  }

  public static class CoralStation {
    public static final double stationLength = Units.inchesToMeters(79.750);

    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
    public static final Pose2d leftOutsideFace =
            new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(291.176),
                Rotation2d.fromDegrees(90 - 144.011)).plus(new Transform2d(stationLength*0.4, 0, Rotation2d.fromDegrees(45)));
    public static final Pose2d rightOutsideFace =
            new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(25.824),
                Rotation2d.fromDegrees(144.011 - 90)).plus(new Transform2d(stationLength*0.4, 0, Rotation2d.fromDegrees(-45)));
    public static final Pose2d leftInsideFace =
                new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(291.176),
                    Rotation2d.fromDegrees(90 - 144.011)).plus(new Transform2d(stationLength*0.4, 0, Rotation2d.fromDegrees(45)));
    public static final Pose2d rightInsideFace =
                new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(25.824),
                    Rotation2d.fromDegrees(144.011 - 90)).plus(new Transform2d(stationLength*0.4, 0, Rotation2d.fromDegrees(-45)));
  }

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefLevel, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
    public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();
    public static final List<Pose2d> algaePositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {
      // Initialize faces
    //   var aprilTagLayout = AprilTagLayoutType.OFFICIAL.getLayout();
    AprilTagFieldLayout  aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

      centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
        Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          var rightBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));
          var leftBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));
            var algaePose =
                      new Pose2d(
                          new Translation2d(
                              poseDirection
                                  .transformBy(new Transform2d(adjustX, 0, new Rotation2d()))
                                  .getX(),
                              poseDirection
                                  .transformBy(new Transform2d(adjustX, 0, new Rotation2d()))
                                  .getY()), 
                          new Rotation2d(
                              poseDirection.getRotation().getRadians()));        

          fillRight.put(level, rightBranchPose);
          fillLeft.put(level, leftBranchPose);
          fillRight2d.put(level, rightBranchPose.toPose2d());
          fillLeft2d.put(level, leftBranchPose.toPose2d());
          algaePositions.add(algaePose);
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
        branchPositions2d.add(fillRight2d);
        branchPositions2d.add(fillLeft2d);
      }
   }
  
    //   for(int i = 0; i < branchPositions.size(); i++) {
    //       for (var level : ReefHeight.values()) {
    //           DogLog.log("Reef Branch Positions "+i, String.format(
    //             "(%.3f, %.3f) %.2f degrees",
    //             branchPositions.get(i).get(level).toPose2d().getX(),
    //             branchPositions.get(i).get(level).toPose2d().getY(),
    //             branchPositions.get(i).get(level).toPose2d().getRotation().getDegrees())
    //             );
    //   }
    // }
    
  
}

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

 

  public enum ReefLevel {
    // L1Inside(0,Units.inchesToMeters(25.0), 0),
    L1(0,Units.inchesToMeters(25.0), 0),
    L1Top(1,Units.inchesToMeters(25.0), 0),
    L2(2,Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L3(3,Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L4(4,Units.inchesToMeters(72), -90);



    ReefLevel(int level, double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // Degrees
      this.level = level ;

    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(reefLevel -> reefLevel.ordinal() == level)
          .findFirst()
          .orElse(L1);
    }

    public final double height;
    public final double pitch;
    public final int level;

  }

  public enum CoralStations {   
    LEFT(0),
    RIGHT(1);

    CoralStations(int side) {
      this.side = side;
    }
    public final int side;

  }

  
  public enum L1DiagonalShootingOffSets {   
    LEFT(19.8,5.5, 25), //INCHES DEGREES
    RIGHT(19.8,5.5, -25);

    L1DiagonalShootingOffSets(double x, double y, double angle) {
      this.x = x;
      this.y = y;
      this.angle = angle;

    }
    public final double x;
    public final double y;
    public final double angle;

  }

  public enum L1VerticalShootingOffSets {   
    LEFTINSIDE(10,5, 0), //INCHES, DEGREES
    RIGHTINSIDE(10,-5, -0),
    RIGHTOUTSIDE(10,5, -0),
    LEFTOUTSIDE(10,-5, -0);


    

    L1VerticalShootingOffSets(double x, double y, double angle) {
      this.x = x;
      this.y = y;
      this.angle = angle;

    }
    public final double x;
    public final double y;
    public final double angle;

  }

  


//   public static Pose2d getReefPoseOffset(DoubleSupplier offsetScalar, int reefFace) {
//     Pose2d facePose = Reef.centerFaces[reefFace];
//     // Transform2d offsetVector = new Transform2d(, , 0.0);
//     Translation2d offsetVectorTranslation2d = new Translation2d(offsetScalar.getAsDouble(), Rotation2d.fromDegrees(180 - (60 * reefFace)));
//     Transform2d offsetVector = new Transform2d(offsetVectorTranslation2d, new Rotation2d(0));
//     Pose2d drivePose = facePose.plus(offsetVector);
//     return drivePose; 
//   }

  public static Pose2d getClosestL1(Supplier<Pose2d> robotPose) {
    int minIndex = 0;
    double minDistance = Double.MAX_VALUE;
    Transform2d currentFacePose = new Transform2d(0.0, 0.0, new Rotation2d(0.0));
    double currentDistance = 0.0;
    for (int i=0; i<6; i++) {
        currentFacePose = AllianceFlipUtil.apply(Reef.centerFaces[i]).minus(robotPose.get());
        currentDistance = currentFacePose.getTranslation().getNorm();
        if(currentDistance < minDistance) {
            minDistance = currentDistance;
            minIndex = i;
        }
    } 
    return AllianceFlipUtil.apply(Reef.centerFaces[minIndex].
    transformBy(Constants.Vision.reefLevelOffsetsMap.get(ReefLevel.L1)));//.getTranslation()), Reef.centerFaces[minIndex].getRotation().plus(Rotatio)));
  }

  /**
   * gets only L2 pose
   * @param robotPose
   * @return
   */
  public static Pose2d getClosestPole(Supplier<Pose2d> robotPose) {
    int minIndex = 0;
    double minDistance = Double.MAX_VALUE;
    Transform2d currentFacePose = new Transform2d(0.0, 0.0, new Rotation2d(0.0));
    double currentDistance = 0.0;
    for (int i=0; i<12; i++) {
        currentFacePose = AllianceFlipUtil.apply(Reef.branchPositions2d.get(i).get(ReefLevel.L2)).minus(robotPose.get());
        currentDistance = currentFacePose.getTranslation().getNorm();
        if(currentDistance < minDistance) {
            minDistance = currentDistance;
            minIndex = i;
        }
    } 
        return AllianceFlipUtil.apply(FieldConstants.Reef.branchPositions.get(minIndex).get(ReefLevel.L2).toPose2d()).plus(Constants.Vision.reefLevelOffsetsMap.get(ReefLevel.L2));

    // return Reef.branchPositions2d.get(minIndex).get(ReefLevel.L2);
  }

  public static boolean getCloseEnoughForAutoDrive(Supplier<Pose2d> robotPose) {
    return robotPose.get().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center)) < AutoDropoff.distanceToAutoDrive + Reef.faceLength + AutoDropoff.robotThickness;
  }

}