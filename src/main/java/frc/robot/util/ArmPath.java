// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/** an arm path from a starting point going through a list of intermediate points */
public class ArmPath {
    public List<ArmPoint> points;

    public ArmPath(List<ArmPoint> points) {
        for (int i = 0; i < points.size()-1; i++) { // interpolate between each point
            points.addAll(ArmPathplannerUtil.interpolateArmPath(points.get(i), points.get(i+1)));
        }
    }

    /** interpolates linearly betwen start, points, and end */
    public ArmPath(List<ArmPoint> points, ArmPoint start, ArmPoint end) {
        this.points.addAll(ArmPathplannerUtil.interpolateArmPath(start, points.get(0)));
        for (int i = 0; i < points.size()-1; i++) {
            this.points.addAll(ArmPathplannerUtil.interpolateArmPath(points.get(i), points.get(i+1)));
        }
        this.points.addAll(ArmPathplannerUtil.interpolateArmPath(points.get(points.size()-1), end));
    }

   

   
}
