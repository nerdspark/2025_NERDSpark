// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.MarshalException;

import edu.wpi.first.math.geometry.Translation2d;
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

    public static final int shoulderMotorPort = 9; // TODO fix these port numbers
    public static final int elbowMotorPort = 10;
    public static final int wristMotorPort = 11;
    public static final int handMotorPort = 12;
    public static final double currentLimitShoulder = 20;
    public static final double shoulderRadPerRot = 2 * Math.PI; //TODO fix with gear ratio

    public static final double currentLimitElbow = 20;
    public static final double elbowRadPerRot = 2 * Math.PI; //TODO fix with gear ratios
    public static final double shoulderOffset = 0.0; // TODO fidn these, radians, fwd = 0
    public static final double elbowOffset = 0.0; // TODO find these, negative of measurement

    //shoulder true offset: 34.513 deg below forward horizontal
    //shoulder gearbox: 25:1
    //shoulder stage 0: 36:26

    // elbow true offset: 122.198 deg above forward horizontal
    // elbow gearbox: 25:1
    //elbow stage 0: 38:26
    // elbow stage 1: 50:50

    //wrist up/down gearbox: 25:1
    //wrist up/down stage 0: 49:50
    //wrist up/down stage 1: 49:49
    //wrist up/down stage 2: 35:49

    //wrist twist gearbox: 25:1
    //wrist twist stage 0: 49:50
    //wrist twist stage 1: 49:49
    //wrist twist stage 2: 35:49
    //wrist twist bevel: 1:1

    public static final double baseStageLength = 23.158;  
    public static final double secondStageLength = 25.475;

  }
  public static class ReefSetPoints {
    public static final Translation2d l1Reef = new Translation2d(2.2, 25.2);
    public static final Translation2d l2Reef = new Translation2d(2.2, 30.2);
    public static final Translation2d l3Reef = new Translation2d(2.2, 35.2);
    public static final Translation2d l4Reef = new Translation2d(2.2, 40.2);
    public static final Translation2d l5Reef = new Translation2d(2.2, 45.2);
  }
  public static class ArmTestAngles{
    public static final double testElbowAngle = Units.degreesToRadians(30);
    public static final double testShoulderAngle = Units.degreesToRadians(30);
  }
  

  public final class ArmGains {
    
      public static final double shoulderP = 90.0; //TODO CHANGE ALL OF THIS
      public static final double shoulderI = 0.0;
      public static final double shoulderD = 5.0;
      public static final double elbowP = 50.0;
      public static final double elbowI = 0.0;
      public static final double elbowD = 3.5;
      public static final double shoulderS = 0.0;
      public static final double shoulderG = 0.1;
      public static final double shoulderV = 0.0;
      public static final double shoulderA = 0.0;
      public static final double elbowS = 0.0;
      public static final double elbowG = 1.05;
      public static final double elbowV = 0.0;
      public static final double elbowA = 0.0;
  }
  public static class WristTestAngles{
    public static final double testWristFlipAngle = Units.degreesToRadians(30);
    public static final double testWristTwistAngle = Units.degreesToRadians(30);
  }
  public static class ArmSetPoints {
    public static final Translation2d home = new Translation2d(0,0); // TODO change this
  }
}
