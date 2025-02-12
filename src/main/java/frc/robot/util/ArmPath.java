// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmMap;

/** Add your docs here. */
public class ArmPath {
    public static Rotation2d ArmPathChooser(List<Translation2d> armPaths,Translation2d position){
        Translation2d closestVector = new Translation2d(10000, 100000);
        for (int i = armPaths.size() - 1; i >= 0; i-- ){
            SmartDashboard.putNumber("distance pos - target", armPaths.get(i).getDistance(position));
            if (armPaths.get(i).getDistance(position) < ArmMap.lookAheadDistance){
                Rotation2d angle = armPaths.get(i).minus(position).getAngle();
                SmartDashboard.putBoolean("on path", true);
                SmartDashboard.putNumber("target point angle", angle.getDegrees());
                return angle;
            }
            if (armPaths.get(i).getDistance(position) < closestVector.getNorm()){
                closestVector = armPaths.get(i).minus(position);
            }
        }
        SmartDashboard.putBoolean("on path", false);
        SmartDashboard.putNumber("target point angle", closestVector.getAngle().getDegrees());
        return closestVector.getAngle();
    }
    public static boolean CheckArmPosition(List<Translation2d> armPaths, Translation2d position){
        return position.getDistance(armPaths.get(armPaths.size()-1)) < ArmMap.endDistance;
    }
}
