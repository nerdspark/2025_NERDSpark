// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Manages a coral list. */
public class CoralArrayManager {

    /** Chooses a coral to target and clears out others. */
    public List<CoralObject> selectCoral(List<CoralObject> corals) {
        ArrayList<DoubleSupplier> distances = new ArrayList<>();
        int sizeCorals = corals.size();
        for (int i = 0; i < sizeCorals - 1; i++) {
            double distanceNew = corals.get(i).getDistance();
            distances.add(() -> distanceNew);
        }
        int sizeDistances = distances.size();
        int indexMinDistance = 0;

        for (int i = 1; i < sizeDistances; i++) {
            double distanceMinCurrent = distances.get(indexMinDistance).getAsDouble();
            double distanceMinProspective = distances.get(i).getAsDouble();

            if (distanceMinProspective < distanceMinCurrent) {
                indexMinDistance = i;
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            if (i != indexMinDistance) {
                CoralObject coralIgnored = corals.get(i);
                coralIgnored.setCoralIgnored(true);
                corals.set(i, coralIgnored);
            } else {
                CoralObject coralTargeted = corals.get(i);
                coralTargeted.setCoralTargeted(true);
                corals.set(i, coralTargeted);
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralChecked = corals.get(i);
            if (coralChecked.getIgnored()) {
                corals.remove(i);
                i--;
                sizeCorals = corals.size();
            }
        }

        return corals;
    }
    /** Filters outdated corals from the list. */
    public List<CoralObject> expiryFilter(List<CoralObject> corals, double hb, double fps) {
        double expiryFrameDiff = fps * 3;
        int sizeCorals = corals.size();

        for(int i = 0; i < sizeCorals; i++) {
            CoralObject coralCompared = corals.get(i);
            double coralHBDiff = Math.abs(coralCompared.getHB() - hb);

            if (coralHBDiff > expiryFrameDiff) {
                CoralObject coralIgnored = corals.get(i);
                coralIgnored.setCoralIgnored(true);
                corals.set(i, coralIgnored);
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralChecked = corals.get(i);
            if (coralChecked.getIgnored()) {
                corals.remove(i);
                i--;
                sizeCorals = corals.size();
            }
        }

        return corals;
    }

    /** Filters out corals that are too close together to be considered distinct, removing the older version. */
    public List<CoralObject> displacementFilter(List<CoralObject> corals) {
        double maxDisplacement = 0.3;
        int sizeCorals = corals.size();

        for (int i = 0; i < sizeCorals - 1; i++) {
            for (int j = i + 1; j < sizeCorals; j++) {
                CoralObject coral1 = corals.get(i);
                CoralObject coral2 = corals.get(j);

                if (!coral1.getIgnored() && !coral2.getIgnored()) {
                    double poseX1 = coral1.getPose().getX();
                    double poseY1 = coral1.getPose().getY();

                    double poseX2 = coral2.getPose().getX();
                    double poseY2 = coral2.getPose().getY();

                    double distance = Math.sqrt(Math.pow((poseX2 - poseX1),2) + Math.pow((poseY2 - poseY1), 2));

                    if (distance <= maxDisplacement) {
                        double hb1 = coral1.getHB();
                        double hb2 = coral2.getHB();

                        if (hb1 < hb2) {
                            CoralObject coralIgnored = coral1;
                            coralIgnored.setCoralIgnored(true);
                            corals.set(i, coralIgnored);
                        } else {
                            CoralObject coralIgnored = coral2;
                            coralIgnored.setCoralIgnored(true);
                            corals.set(j, coralIgnored);
                        }
                    }
                } else if (coral1.getIgnored()) {
                    break;
                }
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralChecked = corals.get(i);
            if (coralChecked.getIgnored()) {
                corals.remove(i);
                i--;
                sizeCorals = corals.size();
            }
        }

        return corals;
    }

    public List<CoralObject> possibilityFilter(List<CoralObject> corals) {
        double fieldXDim = 17.55;
        double fieldYDim = 8.05;

        double borderThickness = 0.1;

        int sizeCorals = corals.size();

        for (int i = 0; i < sizeCorals; i++) {
            Pose2d coralPose = corals.get(i).getPose();

            double coralX = coralPose.getX();
            double coralY = coralPose.getY();

            if ((borderThickness > coralX) || ((fieldXDim - borderThickness) < coralX) || 
            (borderThickness > coralY) || ((fieldYDim - borderThickness) < coralY)) {
                CoralObject coralIgnored = corals.get(i);
                coralIgnored.setCoralIgnored(true);
                corals.set(i, coralIgnored);
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralChecked = corals.get(i);
            if (coralChecked.getIgnored()) {
                corals.remove(i);
                i--;
                sizeCorals = corals.size();
            }
        }

        return corals;
    }

    /** Updates the distance and angle to each coral in the list. */
    public List<CoralObject> distanceAndYawUpdate(List<CoralObject> corals, Pose2d pose) {
        double poseX = pose.getX();
        double poseY = pose.getY();
        
        int sizeCorals = corals.size();

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralToUpdate = corals.get(i);

            Pose2d coralPose = coralToUpdate.getPose();
            
            double coralX = coralPose.getX();
            double coralY = coralPose.getY();

            double distance = Math.sqrt(Math.pow((coralX - poseX), 2) + 
            Math.pow(coralY - poseY, 2));

            Rotation2d coralYawNew = new Rotation2d(Math.atan2(coralY - poseY, coralX - poseX));

            Pose2d updatedCoralPose = new Pose2d(coralX, coralY, coralYawNew);

            coralToUpdate.setCoralPose(updatedCoralPose);
            coralToUpdate.setCoralDistance(distance);

            corals.set(i, coralToUpdate);
        }

        return corals;
    }

    /** Detects if a coral is within a certain distance. */
    public boolean getCoralInRange(List<CoralObject> corals, Pose2d pose) {
        boolean coralInRange = false;
        
        double poseX = pose.getX();
        double poseY = pose.getY();

        int sizeCoral = corals.size();

        double minRange = Units.feetToMeters(10); //2 ft in m

        for (int i = 0; i < sizeCoral; i++) {
            CoralObject coralChecked = corals.get(i);

            Pose2d coralPose = coralChecked.getPose();

            double coralX = coralPose.getX();
            double coralY = coralPose.getY();

            double distance = Math.sqrt(Math.pow((coralX - poseX), 2) + 
            Math.pow(coralY - poseY, 2));

            if (distance <= minRange) {
                coralInRange = true;
            }
        }

        return coralInRange;
    }

    /** Returns the closest coral in the list. */
    public CoralObject getClosestCoral(List<CoralObject> corals) {
        ArrayList<DoubleSupplier> distances = new ArrayList<>();
        int sizeCorals = corals.size();
        for (int i = 0; i < sizeCorals - 1; i++) {
            double distanceNew = corals.get(i).getDistance();
            distances.add(() -> distanceNew);
        }
        int sizeDistances = distances.size();
        int indexMinDistance = 0;

        for (int i = 1; i < sizeDistances; i++) {
            double distanceMinCurrent = distances.get(indexMinDistance).getAsDouble();
            double distanceMinProspective = distances.get(i).getAsDouble();

            if (distanceMinProspective < distanceMinCurrent) {
                indexMinDistance = i;
            }
            
        }

        CoralObject coralClosest = corals.get(indexMinDistance);
        
        return coralClosest;
    }


}
