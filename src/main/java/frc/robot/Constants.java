// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants {

    public static final int shoulderMotorLeftPort = 7; // 7
    public static final int shoulderMotorRightPort = 9; // 9
    public static final int elbowMotorLeftPort = 10;
    public static final int elbowMotorRightPort = 8;
    public static final int wristFlipMotorPort = 13;
    public static final int wristTwistMotorPort = 14;
    public static final int gripperMotorPort = 11;


    public static final double currentLimitShoulder = 30.0;
    public static final double currentLimitElbow = 24.0;
    public static final double currentLimitWristFlip = 20.0; //40.0
    public static final double currentLimitWristTwist = 8.0;
    public static final double currentLimitGripperOpen = 3.0;
    public static final double currentLimitGripperClose = 10.0;
    public static final double gripperPowerClose = 0.1;
    public static final double gripperPowerOpen = -0.04;

    //shoulder true offset: 34.513 deg below forward horizontal
    //shoulder gearbox: 75:1
    //shoulder stage 0: 36:26
    public static final double shoulderGearRatio = 75.0*36.0/26.0;
    
    // elbow true offset: 122.198 deg above forward horizontal
    // elbow gearbox: 75:1
    //elbow stage 0: 38:26
    // elbow stage 1: 50:50
    public static final double elbowGearRatio = 75.0*38.0/26.0;

    //wrist up/down gearbox: 25:1
    //wrist up/down stage 0: 49:50
    //wrist up/down stage 1: 49:49
    //wrist up/down stage 2: 35:49
    public static final double wristFlipGearRatio = 25.0*49.0/50.0*35.0/49.0;
    //wrist twist gearbox: 25:1
    //wrist twist stage 0: 49:50
    //wrist twist stage 1: 49:49
    //wrist twist stage 2: 35:49
    //wrist twist bevel: 1:1
    public static final double wristTwistGearRatio = 25.0*49.0/50.0*35.0/49.0;
    public static final double gripperGearRatio = 5.0*5.0 / 3.0;
    public static final double baseStageLength = 23.158;
    public static final double secondStageLength = 25.475;


    public static final double shoulderRadPerRot = shoulderGearRatio; 
    public static final double elbowRadPerRot = elbowGearRatio; 
    public static final double wristFlipRadPerRot = wristFlipGearRatio;
    public static final double wristTwistRadPerRot = wristTwistGearRatio;
    public static final double gripperRadPerRot = gripperGearRatio;
    public static final double gripperOffset = 0;
    public static final double shoulderOffset = -0.35 / 2.0 / Math.PI; // TODO fidn these, radians, fwd = 0
    public static final double elbowOffset = 2.158 / 2.0 / Math.PI; // TODO find these, negative of measurement
    public static final double wristFlipOffset = (0.38) / 2.0 / Math.PI;
    public static final double wristTwistOffset = -0.35 / 2.0 / Math.PI;

    /** wrist flip belting ratio between elbow and the wrist */
    public static final double wristFlipToElbowRatio = 1.0/(35.0 / 49.0);
    public static final double wristTwistToElbowRatio = 1.0/(35.0 / 49.0);
    public static final double wristTwistToFlipRatio = -1.0 / 1.0;
  }
  // public static class ReefSetPoints {
  //   public static final Translation2d l1Reef = new Translation2d(30.2, 35.2);
  //   public static final Translation2d l2Reef = new Translation2d(2.2, 30.2);
  //   public static final Translation2d l3Reef = new Translation2d(2.2, 35.2);
  //   public static final Translation2d l4Reef = new Translation2d(2.2, 40.2);
  //   public static final Translation2d l5Reef = new Translation2d(2.2, 45.2);
  // }
  public static class ArmTestAngles{
    public static final double testElbowAngle = Units.degreesToRadians(45);
    public static final double testShoulderAngle = Units.degreesToRadians(45);
  }
  public static class ArmPickpup {
    public static final Translation2d armPos = new Translation2d(-15, -15);
    public static final boolean inBend = false;
    public static final double wristFlipPos = Units.degreesToRadians(180.0);
    public static final double wristTwistPos = Units.degreesToRadians(180.0);
    public static final List<Translation2d> home_pickup = List.of(new Translation2d(-14.0, 20.0));
    }

  
  public static class ArmSetpoints {

    public static final Translation2d home = new Translation2d(9.9,14.7); // TODO: tune for backlash and comp bot
    public static final int setPointCount = 7;
    public static double interpolationDistance = 0.1; // inches

    /**
     * contains a list of endpoints
     * @home 0
     * @groundPickup 1
     * @L1Reef 2
     * @L2Reef 3
     * @L3Reef 4
     * @L4Reef 5
     */
    public static ArmPoint[] armSetPoints = new ArmPoint[ArmSetpoints.setPointCount - 1]; 
    static{
      armSetPoints[0] = new ArmPoint(ArmSetpoints.home, false, 0.0, 0.0);
      armSetPoints[1] = new ArmPoint(new Translation2d(46.82, -12.48), Units.degreesToRadians(0), 0.0);
      armSetPoints[2] = new ArmPoint(new Translation2d(5.0, 10.0));
      armSetPoints[3] = new ArmPoint(new Translation2d(5.0, 16.0));
      armSetPoints[4] = new ArmPoint(new Translation2d(23.27, 38.46));
      armSetPoints[5] = new ArmPoint(new Translation2d(5.0, 40.0));
    }

    /**
     * contains a list of intermediate points from first index to second index
     * @home 0
     * @groundPickup 1
     * @L1Reef 2
     * @L2Reef 3
     * @L3Reef 4
     * @L4Reef 5
     */
    @SuppressWarnings("unchecked")
    public static List<ArmPoint>[][] intermediatePoints = new List[ArmSetpoints.setPointCount-1][ArmSetpoints.setPointCount-1]; 
    static{
      intermediatePoints[0][1] = (List<ArmPoint>) List.of(new ArmPoint(new Translation2d(22.43, 30.52)), new ArmPoint(new Translation2d(47.519, 10.114)));

      intermediatePoints[0][2] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[0][3] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[0][4] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[0][5] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
    
      intermediatePoints[1][2] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[1][3] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[1][4] = (List<ArmPoint>) List.of(new ArmPoint(new Translation2d(40.0, 15.0)));
      intermediatePoints[1][5] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));

      intermediatePoints[2][3] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));
      intermediatePoints[2][4] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));
      intermediatePoints[2][5] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));

      intermediatePoints[3][4] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));
      intermediatePoints[3][5] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));

      intermediatePoints[4][5] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));


      intermediatePoints[1][0] = (List<ArmPoint>) List.of(new ArmPoint(new Translation2d(28.43, 22.52)), new ArmPoint(new Translation2d(43.519, 10.114)));

      intermediatePoints[2][0] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[3][0] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[4][0] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[5][0] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
    
      intermediatePoints[2][1] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[3][1] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));
      intermediatePoints[4][1] = (List<ArmPoint>) List.of(new ArmPoint(new Translation2d(12.75, 20.95)));
      intermediatePoints[5][1] = List.of(new ArmPoint(new Translation2d(20.0, 20.0)));

      intermediatePoints[3][2] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));
      intermediatePoints[4][2] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));
      intermediatePoints[5][2] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));

      intermediatePoints[4][3] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));
      intermediatePoints[5][3] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));

      intermediatePoints[5][4] = List.of(new ArmPoint(new Translation2d(-14.0, 20.0)));

      // for (int i = 0; i < ArmSetpoints.setPointCount-1; i++){
      //   for (int j = 0; j < i; j++){
      //     if (intermediatePoints[i][j] == null && intermediatePoints[j][i] != null){
      //       intermediatePoints[i][j] = intermediatePoints[j][i];
      //       Collections.reverse(intermediatePoints[i][j]);
  
      //     } else if (intermediatePoints[i][j] != null && intermediatePoints[j][i] == null){
      //       intermediatePoints[j][i] = intermediatePoints[i][j];
      //       Collections.reverse(intermediatePoints[j][i]);
      //     }
      //   } 
      // }
    }
    /**
     * contains a list of intermediate points from first index to second index with wrist twsit/flip movement
     * @home 0
     * @groundPickup 1
     * @L1Reef 2
     * @L2Reef 3
     * @L3Reef 4
     * @L4Reef 5
     */
    // public static final List<ArmPoint>[][] wristIntermediatePoints = new List[ArmSetpoints.setPointCount-1][ArmSetpoints.setPointCount-1]; {
    //   wristIntermediatePoints[0][1] = List.of(new ArmPoint(new Translation2d( 20, 20), false, 30, 30));
    //   wristIntermediatePoints[0][2] = List.of(new ArmPoint(new Translation2d( 20, 20), false, 30, 30));
    //   wristIntermediatePoints[0][3] = List.of(new ArmPoint(new Translation2d( 20, 20), false, 30, 30));
    //   wristIntermediatePoints[0][4] = List.of(new ArmPoint(new Translation2d( 20, 20), false, 30, 30));
    //   wristIntermediatePoints[0][5] = List.of(new ArmPoint(new Translation2d( 20, 20), false, 30, 30));
    //   for (int i = 0; i < ArmSetpoints.setPointCount-1; i++){
    //     for (int j = 0; j < ArmSetpoints.setPointCount-1; i++){

    //     }
    //   }
    

    /**
     * contains a full path between two points
     * @home 0
     * @groundPickup 1
     * @L1Reef 2
     * @L2Reef 3
     * @L3Reef 4
     * @L4Reef 5
     */
    public static ArmPath[][] armPaths = new ArmPath[ArmSetpoints.setPointCount-1][ArmSetpoints.setPointCount-1]; 
    static {
      for (int i = 0; i < ArmSetpoints.setPointCount-1; i++){
        for (int j = 0; j < ArmSetpoints.setPointCount-1; j++){
          if (i != j){
            armPaths[i][j] = new ArmPath(intermediatePoints[i][j], armSetPoints[i], armSetPoints[j]);
            System.out.println("Created arm path" + i + " " + j);
            System.out.println(armPaths[i][j]);
          } else { // set path to just the endpoint
            armPaths[i][j] = new ArmPath(List.of(armSetPoints[i]));
          }
        }
      }
    }
    
  }
  

  public final class ArmGains {
      public static final double shoulderP = 50.0; //TODO CHANGE SOME OF THIS LATER //52.0
      public static final double shoulderI = 0.0;
      public static final double shoulderD = 0.0;
      public static final double elbowP = 20.0;//20.0
      public static final double elbowI = 0.0;
      public static final double elbowD = 0.0;
      public static final double wristFlipP = 20.0; //20.0
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
    public static final double shoulderP = 70.0; //TODO CHANGE SOME OF THIS LATER //52.0
      public static final double shoulderI = 0.0;
      public static final double shoulderD = 0.0;
      public static final double elbowP = 20.0;//20.0
      public static final double elbowI = 0.0;
      public static final double elbowD = 0.0;
      public static final double wristFlipP = 20.0; //20.0
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
  public static class WristTestAngles{
    public static final double testWristFlipAngle = Units.degreesToRadians(90.0);
    public static final double testWristTwistAngle = Units.degreesToRadians(90.0);
    public static final double testGripperAngle = Units.degreesToRadians(30);
  }

  public static class ArmMap {
    public static final ArmPath testPath = new ArmPath(List.of(new ArmPoint(new Translation2d(7.7, 13.3)), new ArmPoint(new Translation2d(15.7, 31.3)), new ArmPoint(new Translation2d(25.7, 39.3)), new ArmPoint(new Translation2d(35.7, 31.3))));
    public static final double lookAheadDistance = 12.0;
    public static final double endDistance = 7.0;
    public static final double linearApproximationTime = 0.05; // seconds
    public static final double velocity = 14;
    public static final double maxMotorVelocity = 10;
  }
}

