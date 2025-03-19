// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.time.chrono.MinguoDate;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ArmVelocityGains;

/** Add your docs here. */
public class ArmPathplannerUtil {
    /** chooses next point on a 2d path given current arm position
     * @param armPaths list of arm points
     * @param position current arm position
     */
    public static Rotation2d ArmPathChooser(List<Translation2d> armPaths,Translation2d position){
        Translation2d closestVector = new Translation2d(10000, 100000);
        for (int i = armPaths.size() - 1; i >= 0; i-- ){
            // SmartDashboard.putNumber("distance pos - target", armPaths.get(i).getDistance(position));
            if (armPaths.get(i).getDistance(position) < ArmVelocityGains.lookAheadDistance){
                Rotation2d angle = armPaths.get(i).minus(position).getAngle();
                // SmartDashboard.putBoolean("on path", true);
                // SmartDashboard.putNumber("target point angle", angle.getDegrees());
                return angle;
            }
            if (armPaths.get(i).getDistance(position) < closestVector.getNorm()){
                closestVector = armPaths.get(i).minus(position);
            }
        }
        // SmartDashboard.putBoolean("on path", false);
        // SmartDashboard.putNumber("target point angle", closestVector.getAngle().getDegrees());
        return closestVector.getAngle();
    }
    public static ArmPoint getNextPoint(List<ArmPoint> armPaths, ArmPoint currentPoint) {
        ArmPoint closestArmPoint = new ArmPoint(new Translation2d(1000000,1000000));
        for (int i = armPaths.size() - 1; i >= 0; i-- ){
            // SmartDashboard.putNumber("distance pos - target", armPaths.get(i).position.getDistance(position));
            if (armPaths.get(i).position.getDistance(currentPoint.position) < ArmVelocityGains.lookAheadDistance){
                if (armPaths.get(i).inBend == currentPoint.inBend) {
                    return armPaths.get(i);
                } else {
                    if (armPaths.get(i).position.getDistance(currentPoint.position) < ArmVelocityGains.lookAheadDistanceBeforeInflecting) {
                        return armPaths.get(i);
                    }
                }
                // Rotation2d angle = armPaths.get(i).position.minus(position).getAngle();
                // SmartDashboard.putBoolean("on path", true);
                // SmartDashboard.putNumber("target point angle", angle.getDegrees());
            }
            if (armPaths.get(i).position.getDistance(currentPoint.position) < closestArmPoint.position.getNorm()){
                closestArmPoint = armPaths.get(i);
            }
        }
        return closestArmPoint;
    }
    public static int getNextPointIndex(List<ArmPoint> armPaths, Translation2d position) {
        ArmPoint closestArmPoint = new ArmPoint(new Translation2d(1000000,1000000));
        int index = 0;
        for (int i = armPaths.size() - 1; i >= 0; i-- ){
            // SmartDashboard.putNumber("distance pos - target", armPaths.get(i).position.getDistance(position));
            if (armPaths.get(i).position.getDistance(position) < ArmVelocityGains.lookAheadDistance){
                Rotation2d angle = armPaths.get(i).position.minus(position).getAngle();
                // SmartDashboard.putBoolean("on path", true);
                // SmartDashboard.putNumber("target point angle", angle.getDegrees());
                return i;
            }
            if (armPaths.get(i).position.getDistance(position) < closestArmPoint.position.getNorm()){
                closestArmPoint = armPaths.get(i);
                index = i;
            }
        }
        return index;
    }
   

    /** checks if the arm is at the end of the path */
    public static boolean CheckArmPosition(List<ArmPoint> armPaths, ArmPoint position){
        return position.position.getDistance(armPaths.get(armPaths.size()-1).position) < ArmVelocityGains.endDistance && position.inBend == armPaths.get(armPaths.size()-1).inBend;
    }
    /** interpolates linearly between start and end, includes endpoint but not startpoint */
    public static List<ArmPoint> interpolateArmPath(ArmPoint start, ArmPoint end){
        List<ArmPoint> path = (List<ArmPoint>) new ArrayList<ArmPoint>();
        // path.add(start);

        //interpolate polarly every 1 degree
        Rotation2d step = Rotation2d.fromDegrees(ArmVelocityGains.interpolationAngle);
        int pointCount = (int) ((end.position.getAngle().minus(start.position.getAngle())).getDegrees() / ArmVelocityGains.interpolationAngle);
        double stepDist = (end.position.getNorm() - start.position.getNorm()) / (pointCount);

        if (pointCount < 0) {
            pointCount = -pointCount;
            step = step.times(-1);
            stepDist *= -1;
        }
        if (pointCount > 3 || end.position.getDistance(start.position) < 10) { // continue interpolation polarly
            for (int i = 1; i <= pointCount; i++){
                boolean inBend = i < pointCount/2 ? start.inBend : end.inBend;
                path.add(new ArmPoint(new Translation2d(start.position.getNorm() + (stepDist * i), start.position.getAngle().plus(step.times(i))), inBend));
            }
        } else { // switch to linear interpolation
            double distance = start.position.getDistance(end.position);
            Translation2d stepLinear = end.position.minus(start.position).div(distance).times(ArmVelocityGains.interpolationDistance);
            int pointCountLinear = (int)(distance/ArmVelocityGains.interpolationDistance);
            for (int i = 1; i <= pointCountLinear; i++){
                boolean inBend = i < pointCountLinear/2 ? start.inBend : end.inBend;
                path.add(new ArmPoint(start.position.plus(stepLinear.times(i)), inBend));
            }
        }

        path.add(end);
        
        //TODO: add wrist interpolation or other way to time wrist movement

        return path;
    }

    // /** interpolates linearly between all points */
    // public static List<ArmPoint> interpolateArmPath(List<ArmPoint> points){
    //     List<ArmPoint> path = (List<ArmPoint>) new ArrayList<ArmPoint>();
    //     for (int i = 0; i < points.size()+1; i++) {
    //         path.addAll(interpolateArmPath(points.get(i), points.get(i+1)));
    //     }
    //     return path;
    // }

    /** returns the closest armPoint to the current armPosition 
     * @param armPoints possible arm setpoints
     * @param armPosition current arm position
    */
    public static int closestArmPoint(ArmPoint[] armPoints, Translation2d armPosition) {
        Translation2d position = armPosition;
        double minDist = 1000000;
        int closestPoint = 0;
        for (int i = 0; i < armPoints.length; i++) {
            ArmPoint point = armPoints[i];
            if (point.position.getDistance(position) < minDist) {
                minDist = point.position.getDistance(position);
                closestPoint = i;
            }
        }
        // SmartDashboard.putNumber("closest point", closestPoint);
        return closestPoint;
    }

}
