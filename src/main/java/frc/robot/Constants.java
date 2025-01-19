// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class TeleopAutomationConstants {
    public static final double PATH_VELOCITY_MAX = 5.0;
    public static final double PATH_ACCELERATION_MAX = 3.5;
    public static final double PATH_ANGULAR_VELOCITY_MAX = Units.degreesToRadians(360);
    public static final double PATH_ANGULAR_ACCELERATION_MAX = Units.degreesToRadians(540);
  }
}
