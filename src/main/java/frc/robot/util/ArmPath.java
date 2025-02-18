// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/** an interpolated arm path between a starting point, a list of intermediate points, and an end point */
public class ArmPath {
    public List<ArmPoint> points;
    //TODO: add and incorporate optional inflection point
    
    /** interpolates linearly betwen points */
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

    /** returns arm setpoints in a translation2d list */
    public List<Translation2d> getTranslations() {
        List<Translation2d> translations = new ArrayList<Translation2d>();
        for (ArmPoint point : points) {
            translations.add(point.position);
        }
        return translations;
    }

   

   
}
