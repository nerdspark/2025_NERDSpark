// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.subsystems.Inventory;

/** Add your docs here. */
public class PIDToPosition {
    public Translation2d ChooseVector(Pose2d position, boolean inventory) {
        if (inventory == true){
            Translation2d[] positions = new Translation2d[3];
            positions[0] = AutoDriveConstants.position1;
            positions[1] = AutoDriveConstants.position2;
            positions[2] = AutoDriveConstants.position3;
            double[] differences = new double[3];
            differences[0] = position.getTranslation().getDistance(AutoDriveConstants.position1);
            differences[1] = position.getTranslation().getDistance(AutoDriveConstants.position2);
            differences[2] = position.getTranslation().getDistance(AutoDriveConstants.position3);
            SmartDashboard.putNumber("1, 0", differences[0]);
            SmartDashboard.putNumber("-1, 0", differences[1]);
            SmartDashboard.putNumber("0, 1", differences[2]);
            double bestDistance = differences[1];
            int selecter = 0;
            for (int i = 0; i < differences.length; i++){
                if(differences[i] < bestDistance){
                    bestDistance = differences[i];
                    selecter = i;
                }
            }
            return positions[selecter];
        }
        return position.getTranslation();
    }
    public boolean InsideRange(Supplier<Pose2d> position){
        if (ChooseVector(position.get(), true).getDistance(position.get().getTranslation()) < 2){
            return true;
        }else{
            return false;
        }
    }
    
}
