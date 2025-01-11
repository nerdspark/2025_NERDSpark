// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static InterpolatingDoubleTreeMap joystickMap = new InterpolatingDoubleTreeMap();
    static {
      // Key: radial joystick distance
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
    }
    }
