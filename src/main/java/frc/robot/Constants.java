// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.MarshalException;

import edu.wpi.first.math.geometry.Translation2d;

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

    public static final double baseStageLength = 0;  // TODO fix these lengths
    public static final double secondStageLength = 0;

    public static final double currentLimitShoulder = 40;
    public static final double shoulderRadPerRot = 2 * Math.PI; //TODO fix with gear ratio

    public static final double currentLimitElbow = 40;
    public static final double elbowRadPerRot = 2 * Math.PI; //TODO fix with gear ratios
  
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

  }

  public static class ArmSetPoints {
    public static final Translation2d home = new Translation2d(0,0); // TODO change this
  }
}
