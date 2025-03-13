// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class CoralArrayManager {

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

    public List<CoralObject> expiryFilter(List<CoralObject> corals, double hb, double fps) {
        double expiryFrameDiff = fps * 5;
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
}
