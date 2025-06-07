// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Manages a algae list. */
public class AlgaeArrayManager {

    /** Chooses a algae to target and clears out others. */
    public List<AlgaeObject> selectAlgae(List<AlgaeObject> algae) {
        ArrayList<DoubleSupplier> distances = new ArrayList<>();
        int sizeAlgae = algae.size();
        for (int i = 0; i < sizeAlgae - 1; i++) {
            double distanceNew = algae.get(i).getDistance();
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

        for (int i = 0; i < sizeAlgae; i++) {
            if (i != indexMinDistance) {
                AlgaeObject algaeIgnored = algae.get(i);
                algaeIgnored.setAlgaeIgnored(true);
                algae.set(i, algaeIgnored);
            } else {
                AlgaeObject algaeTargeted = algae.get(i);
                algaeTargeted.setAlgaeTargeted(true);
                algae.set(i, algaeTargeted);
            }
        }

        for (int i = 0; i < sizeAlgae; i++) {
            AlgaeObject algaeChecked = algae.get(i);
            if (algaeChecked.getIgnored()) {
                algae.remove(i);
                i--;
                sizeAlgae = algae.size();
            }
        }

        return algae;
    }
    /** Filters outdated algae from the list. */
    public List<AlgaeObject> expiryFilter(List<AlgaeObject> algae, double hb, double fps) {
        double expiryFrameDiff = fps * 5;
        int sizeAlgae = algae.size();

        for(int i = 0; i < sizeAlgae; i++) {
            AlgaeObject algaeCompared = algae.get(i);
            double algaeHBDiff = Math.abs(algaeCompared.getHB() - hb);

            if (algaeHBDiff > expiryFrameDiff) {
                AlgaeObject algaeIgnored = algae.get(i);
                algaeIgnored.setAlgaeIgnored(true);
                algae.set(i, algaeIgnored);
            }
        }

        for (int i = 0; i < sizeAlgae; i++) {
            AlgaeObject algaeChecked = algae.get(i);
            if (algaeChecked.getIgnored()) {
                algae.remove(i);
                i--;
                sizeAlgae = algae.size();
            }
        }

        return algae;
    }

    /** Filters out algae that are too close together to be considered distinct, removing the older version. */
    public List<AlgaeObject> displacementFilter(List<AlgaeObject> algae) {
        double maxDisplacement = 0.3;
        int sizeAlgae = algae.size();

        for (int i = 0; i < sizeAlgae - 1; i++) {
            for (int j = i + 1; j < sizeAlgae; j++) {
                AlgaeObject algae1 = algae.get(i);
                AlgaeObject algae2 = algae.get(j);

                if (!algae1.getIgnored() && !algae2.getIgnored()) {
                    double poseX1 = algae1.getPose().getX();
                    double poseY1 = algae1.getPose().getY();

                    double poseX2 = algae2.getPose().getX();
                    double poseY2 = algae2.getPose().getY();

                    double distance = Math.sqrt(Math.pow((poseX2 - poseX1),2) + Math.pow((poseY2 - poseY1), 2));

                    if (distance <= maxDisplacement) {
                        double hb1 = algae1.getHB();
                        double hb2 = algae2.getHB();

                        if (hb1 < hb2) {
                            AlgaeObject algaeIgnored = algae1;
                            algaeIgnored.setAlgaeIgnored(true);
                            algae.set(i, algaeIgnored);
                        } else {
                            AlgaeObject algaeIgnored = algae2;
                            algaeIgnored.setAlgaeIgnored(true);
                            algae.set(j, algaeIgnored);
                        }
                    }
                } else if (algae1.getIgnored()) {
                    break;
                }
            }
        }

        for (int i = 0; i < sizeAlgae; i++) {
            AlgaeObject algaeChecked = algae.get(i);
            if (algaeChecked.getIgnored()) {
                algae.remove(i);
                i--;
                sizeAlgae = algae.size();
            }
        }

        return algae;
    }

    public List<AlgaeObject> possibilityFilter(List<AlgaeObject> algae) {
        double fieldXDim = 17.55;
        double fieldYDim = 8.05;

        double borderThickness = 0.5;

        int sizeAlgae = algae.size();

        for (int i = 0; i < sizeAlgae; i++) {
            Pose2d algaePose = algae.get(i).getPose();

            double algaeX = algaePose.getX();
            double algaeY = algaePose.getY();

            if ((borderThickness > algaeX) || ((fieldXDim - borderThickness) < algaeX) || 
            (borderThickness > algaeY) || ((fieldYDim - borderThickness) < algaeY)) {
                AlgaeObject algaeIgnored = algae.get(i);
                algaeIgnored.setAlgaeIgnored(true);
                algae.set(i, algaeIgnored);
            }
        }

        for (int i = 0; i < sizeAlgae; i++) {
            AlgaeObject algaeChecked = algae.get(i);
            if (algaeChecked.getIgnored()) {
                algae.remove(i);
                i--;
                sizeAlgae = algae.size();
            }
        }

        return algae;
    }

    /** Updates the distance and angle to each algae in the list. */
    public List<AlgaeObject> distanceAndYawUpdate(List<AlgaeObject> algae, Pose2d pose) {
        double poseX = pose.getX();
        double poseY = pose.getY();
        
        int sizeAlgae = algae.size();

        for (int i = 0; i < sizeAlgae; i++) {
            AlgaeObject algaeToUpdate = algae.get(i);

            Pose2d algaePose = algaeToUpdate.getPose();
            
            double algaeX = algaePose.getX();
            double algaeY = algaePose.getY();

            double distance = Math.sqrt(Math.pow((algaeX - poseX), 2) + 
            Math.pow(algaeY - poseY, 2));

            Rotation2d algaeYawNew = new Rotation2d(Math.atan2(algaeY - poseY, algaeX - poseX));

            Pose2d updatedAlgaePose = new Pose2d(algaeX, algaeY, algaeYawNew);

            algaeToUpdate.setAlgaePose(updatedAlgaePose);
            algaeToUpdate.setAlgaeDistance(distance);

            algae.set(i, algaeToUpdate);
        }

        return algae;
    }

    /** Detects if a algae is within a certain distance. */
    public boolean getAlgaeInRange(List<AlgaeObject> algae, Pose2d pose) {
        boolean algaeInRange = false;
        
        double poseX = pose.getX();
        double poseY = pose.getY();

        int sizeAlgae = algae.size();

        double maxRange = 1.52; //5 ft in m

        for (int i = 0; i < sizeAlgae; i++) {
            AlgaeObject algaeChecked = algae.get(i);

            Pose2d algaePose = algaeChecked.getPose();

            double algaeX = algaePose.getX();
            double algaeY = algaePose.getY();

            double distance = Math.sqrt(Math.pow((algaeX - poseX), 2) + 
            Math.pow(algaeY - poseY, 2));

            if (distance <= maxRange) {
                algaeInRange = true;
            }
        }

        return algaeInRange;
    }

    /** Returns the closest algae in the list. */
    public AlgaeObject getClosestAlgae(List<AlgaeObject> algae) {
        ArrayList<DoubleSupplier> distances = new ArrayList<>();
        int sizeAlgae = algae.size();
        for (int i = 0; i < sizeAlgae - 1; i++) {
            double distanceNew = algae.get(i).getDistance();
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

        AlgaeObject algaeClosest = algae.get(indexMinDistance);
        
        return algaeClosest;
    }

    public void algaeMap(List<AlgaeObject> algae, Field2d field) {
        int sizeAlgae = algae.size();

        for (int i = 0; i < sizeAlgae; i++) {
            AlgaeObject algaeMapped = algae.get(i);
            field.getObject("Algae " + i).setPose(algaeMapped.getPose());
        }
    }


}
