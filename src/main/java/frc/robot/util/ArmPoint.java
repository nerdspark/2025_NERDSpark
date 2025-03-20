// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.Arm;

/** a single setpoint for an arm position */
public class ArmPoint {
    public Translation2d position;
    public boolean inBend = false;
    public double wrist = Units.degreesToRadians(110);
    public ArmPoint(Translation2d point, boolean inBend, double wrist) {
        this.position = point;
        // if (position.getNorm() > ArmConstants.totalStageLength) {
        //     position = new Translation2d(ArmConstants.totalStageLength, position.getAngle());
        // } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
        //     position = new Translation2d(ArmSetpoints.home.getNorm(), position.getAngle());
        // }
        this.inBend = inBend;
        this.wrist = wrist;
    }
    public ArmPoint(Translation2d point, boolean inBend) {
        this.position = point;
        // if (position.getNorm() > ArmConstants.totalStageLength) {
        //     position = new Translation2d(ArmConstants.totalStageLength, position.getAngle());
        // } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
        //     position = new Translation2d(ArmSetpoints.home.getNorm(), position.getAngle());
        // }
        this.inBend = inBend;
    }
    public ArmPoint(Translation2d point, double wrist) {
        this.position = point;
        // if (position.getNorm() > ArmConstants.totalStageLength) {
        //     position = new Translation2d(ArmConstants.totalStageLength, position.getAngle());
        // } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
        //     position = new Translation2d(ArmSetpoints.home.getNorm(), position.getAngle());
        // }
        this.wrist = wrist;
    }
    public ArmPoint(Translation2d point) {
        this.position = point;
        // if (position.getNorm() > ArmConstants.totalStageLength) {
        //     position = new Translation2d(ArmConstants.totalStageLength, position.getAngle());
        // } else if (position.getNorm() < ArmSetpoints.home.getNorm()) {
        //     position = new Translation2d(ArmSetpoints.home.getNorm(), position.getAngle());
        // }
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
    public ArmPoint addToWristFlip(double wristAdd) {
        return new ArmPoint(position, inBend, wrist + wristAdd);
    }
    public ArmPoint add(Translation2d add) {
        return new ArmPoint(position.plus(add), inBend, wrist);
    }
    /**
     * Interpolates between two ArmPoints
     * @param endPoint position to go to at alpha = 1
     * @param alpha percentage of the way to the end point
     * @return the interpolated ArmPoint
     */
    public ArmPoint interpolate(ArmPoint endPoint, double alpha) {
        return new ArmPoint(position.plus(endPoint.position.minus(position).times(alpha)), alpha < ((ArmConstants.totalStageLength - endPoint.position.getNorm())/(2*ArmConstants.totalStageLength - position.getNorm() - endPoint.position.getNorm())) ? inBend : endPoint.inBend, wrist + (alpha * (endPoint.wrist - wrist)));
    }
    public ArmPoint rotateBy(Rotation2d rotateBy) {
        return new ArmPoint(position.rotateBy(rotateBy), inBend, wrist);
    }
    public ArmPoint rotateElbowBy(Rotation2d rotateBy) {
        double distance = position.getNorm();
        double BaseAngleArmDiff = Math.acos(((distance * distance)
                        + (ArmConstants.baseStageLength * ArmConstants.baseStageLength)
                        - (ArmConstants.secondStageLength * ArmConstants.secondStageLength))
                / (2 * distance * ArmConstants.baseStageLength));
        double SecondAngleArmDiff = Math.acos(((distance * distance)
                        - (ArmConstants.baseStageLength * ArmConstants.baseStageLength)
                        + (ArmConstants.secondStageLength * ArmConstants.secondStageLength))
                / (2 * distance * ArmConstants.secondStageLength));
        double shoulderPosition = position.getAngle().getRadians() + (BaseAngleArmDiff * (inBend ? 1 : -1));
        double elbowPosition = position.getAngle().getRadians() + (SecondAngleArmDiff * (inBend ? -1 : 1));
        elbowPosition += rotateBy.getRadians();
        Translation2d newPos = new Translation2d(ArmConstants.baseStageLength, new Rotation2d(shoulderPosition)).plus(new Translation2d(ArmConstants.secondStageLength, new Rotation2d(elbowPosition)));
        return new ArmPoint(newPos, inBend, wrist);
    }

    public ArmPoint withGripperOffset(Translation2d offset) {
        return new ArmPoint(position.minus(offset.rotateBy(new Rotation2d(wrist))), inBend, wrist);
    }
}
