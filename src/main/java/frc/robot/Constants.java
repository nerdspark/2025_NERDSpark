// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.MarshalException;
import java.util.ArrayList;
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


    public static final double currentLimitShoulder = 15.0;
    public static final double currentLimitElbow = 10.0;
    public static final double currentLimitWristFlip = 40.0;
    public static final double currentLimitWristTwist = 20.0;
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

    //wrist up/down gearbox: 9:1
    //wrist up/down stage 0: 49:50
    //wrist up/down stage 1: 49:49
    //wrist up/down stage 2: 35:49
    public static final double wristFlipGearRatio = 9.0*49.0/50.0*35.0/49.0;
    //wrist twist gearbox: 9:1
    //wrist twist stage 0: 49:50
    //wrist twist stage 1: 49:49
    //wrist twist stage 2: 35:49
    //wrist twist bevel: 1:1
    public static final double wristTwistGearRatio = 9.0*49.0/50.0*35.0/49.0;
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
  public static class ReefSetPoints {
    public static final Translation2d l1Reef = new Translation2d(30.2, 35.2);
    public static final Translation2d l2Reef = new Translation2d(2.2, 30.2);
    public static final Translation2d l3Reef = new Translation2d(2.2, 35.2);
    public static final Translation2d l4Reef = new Translation2d(2.2, 40.2);
    public static final Translation2d l5Reef = new Translation2d(2.2, 45.2);
  }
  public static class ArmTestAngles{
    public static final double testElbowAngle = Units.degreesToRadians(45);
    public static final double testShoulderAngle = Units.degreesToRadians(45);
  }
  

  public final class ArmGains {
    
      public static final double shoulderP = 52.0; //TODO CHANGE SOME OF THIS LATER //52.0
      public static final double shoulderI = 0.0;
      public static final double shoulderD = 0.0;
      public static final double elbowP = 20.0;//20.0
      public static final double elbowI = 0.0;
      public static final double elbowD = 0.0;
      public static final double wristFlipP = 20.0; //45.0
      public static final double wristFlipI = 0.0;
      public static final double wristFlipD = 0.0;
      public static final double wristTwistP = 15.0; //30.0
      public static final double wristTwistI = 0.0;
      public static final double wristTwistD = 0.0;
      public static final double gripperP = 10.0; // 10.0
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
  public static class ArmSetPoints {
    public static final Translation2d home = new Translation2d(0,0); // TODO change this
  }
  public static class ArmMap {
    public static final double lookAheadDistance = 3.0;
    public static final List<Translation2d> armPaths = List.of(
    new Translation2d(7.7, 13.3), 
    new Translation2d(7.7, 14.3), 
    new Translation2d(7.7, 15.3), 
    new Translation2d(7.7, 16.3), 
    new Translation2d(7.7, 17.3), 
    new Translation2d(7.7, 18.3), 
    new Translation2d(7.7, 19.3), 
    new Translation2d(7.7, 20.3), 
    new Translation2d(7.7, 21.3), 
    new Translation2d(7.7, 22.3),
    new Translation2d(7.7, 23.3), 
    new Translation2d(7.7, 24.3), 
    new Translation2d(7.7, 25.3), 
    new Translation2d(7.7, 26.3), 
    new Translation2d(7.7, 27.3), 
    new Translation2d(7.7, 28.3), 
    new Translation2d(7.7, 29.3), 
    new Translation2d(7.7, 30.3), 
    new Translation2d(7.7, 31.3), 
    new Translation2d(7.7, 32.3),
    new Translation2d(7.7, 33.3), 
    new Translation2d(7.7, 34.3), 
    new Translation2d(7.7, 35.3), 
    new Translation2d(7.7, 36.3), 
    new Translation2d(7.7, 37.3), 
    new Translation2d(7.7, 38.3), 
    new Translation2d(7.7, 39.3), 
    new Translation2d(8.7, 39.3),
    new Translation2d(9.7, 39.3),
    new Translation2d(10.7, 39.3),
    new Translation2d(11.7, 39.3),
    new Translation2d(12.7, 39.3),
    new Translation2d(13.7, 39.3),
    new Translation2d(14.7, 39.3),
    new Translation2d(15.7, 39.3),
    new Translation2d(16.7, 39.3),
    new Translation2d(17.7, 39.3),
    new Translation2d(18.7, 39.3),
    new Translation2d(19.7, 39.3),
    new Translation2d(20.7, 39.3),
    new Translation2d(21.7, 39.3),
    new Translation2d(22.7, 39.3),
    new Translation2d(23.7, 39.3),
    new Translation2d(24.7, 39.3),
    new Translation2d(25.7, 39.3),
    new Translation2d(26.7, 38.3),
    new Translation2d(27.7, 37.3),
    new Translation2d(28.7, 36.3),
    new Translation2d(29.7, 35.3),
    new Translation2d(30.7, 34.3),
    new Translation2d(31.7, 33.3),
    new Translation2d(32.7, 32.3),
    new Translation2d(33.7, 31.3),
    new Translation2d(34.7, 30.3),
    new Translation2d(35.7, 29.3));
    public static final double endDistance = 5.0;
    public static final double linearApproximationTime = 0.1; // seconds
    public static final double velocity = 3;
    public static final double maxMotorVelocity = 0.1;
  }
}
