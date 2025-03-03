// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.Arm;

/** a single setpoint for an arm position */
public class ArmPoint {
    public Translation2d position;
    public boolean inBend = false;
    public double wristFlip, wristTwist = 0;
    public ArmPoint(Translation2d point, boolean inBend, double wristFlip, double wristTwist) {
        this.position = point;
        if (position.getNorm() > ArmConstants.totalStageLength) {
            position = new Translation2d(ArmConstants.totalStageLength, point.getAngle());
        } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
            position = new Translation2d(ArmSetpoints.home.getNorm(), point.getAngle());
        }
        this.inBend = inBend;
        this.wristFlip = wristFlip;
        this.wristTwist = wristTwist;
    }
    public ArmPoint(Translation2d point, boolean inBend) {
        this.position = point;
        if (position.getNorm() > ArmConstants.totalStageLength) {
            position = new Translation2d(ArmConstants.totalStageLength, point.getAngle());
        } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
            position = new Translation2d(ArmSetpoints.home.getNorm(), point.getAngle());
        }
        this.inBend = inBend;
    }
    public ArmPoint(Translation2d point, double wristFlip, double wristTwist) {
        this.position = point;
        if (position.getNorm() > ArmConstants.totalStageLength) {
            position = new Translation2d(ArmConstants.totalStageLength, point.getAngle());
        } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
            position = new Translation2d(ArmSetpoints.home.getNorm(), point.getAngle());
        }
        this.wristFlip = wristFlip;
        this.wristTwist = wristTwist;
    }
    public ArmPoint(Translation2d point) {
        this.position = point;
        if (position.getNorm() > ArmConstants.totalStageLength) {
            position = new Translation2d(ArmConstants.totalStageLength, point.getAngle());
        } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
            position = new Translation2d(ArmSetpoints.home.getNorm(), point.getAngle());
        }
    }
    public static List<ArmPoint> fromTranslations(List<Translation2d> points, boolean inBend) {
        List<ArmPoint> ret = new ArrayList<>();
        for (int i = 0; i < points.size(); i++) {
            ret.add(new ArmPoint(points.get(i), inBend));
        }
        return ret;
    }
}
