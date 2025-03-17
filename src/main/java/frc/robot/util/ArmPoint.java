// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.Arm;

/** a single setpoint for an arm position */
public class ArmPoint {
    public Translation2d position;
    public boolean inBend = false;
    public double wrist = 0;
    public ArmPoint(Translation2d point, boolean inBend, double wrist) {
        this.position = point;
        if (position.getNorm() > ArmConstants.totalStageLength) {
            position = new Translation2d(ArmConstants.totalStageLength, point.getAngle());
        } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
            position = new Translation2d(ArmSetpoints.home.getNorm(), point.getAngle());
        }
        this.inBend = inBend;
        this.wrist = wrist;
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
    public ArmPoint(Translation2d point, double wrist) {
        this.position = point;
        if (position.getNorm() > ArmConstants.totalStageLength) {
            position = new Translation2d(ArmConstants.totalStageLength, point.getAngle());
        } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
            position = new Translation2d(ArmSetpoints.home.getNorm(), point.getAngle());
        }
        this.wrist = wrist;
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
    public ArmPoint withWrist(double wristPos) {
        return new ArmPoint(position, inBend, wristPos);
    }
    public ArmPoint flipBy(double wristAdd) {
        return new ArmPoint(position, inBend, wrist + wristAdd);
    }
    public ArmPoint add(Translation2d add) {
        return new ArmPoint(position.plus(add), inBend, wrist);
    }
    public ArmPoint rotateBy(Rotation2d rotateBy) {
        return new ArmPoint(position.rotateBy(rotateBy), inBend, wrist);
    }
}
