// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class AlgaeObject {
    private Pose2d pose;
    private double hb;
    private double distance;
    private boolean targeted;
    private boolean ignored;
    
    public AlgaeObject(Pose2d pose, double hb, double distance, boolean targeted, boolean ignored) {
        this.pose = pose;
        this.hb = hb;
        this.distance = distance;
        this.targeted = targeted;
        this.ignored = ignored;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setAlgaePose(Pose2d pose) {
        this.pose = pose;
    }

    public double getHB() {
        return hb;
    }

    public void setAlgaeHB(double hb) {
        this.hb = hb;
    }

    public double getDistance() {
        return distance;
    }

    public void setAlgaeDistance(double distance) {
        this.distance = distance;
    }

    public boolean getTargeted() {
        return targeted;
    }

    public void setAlgaeTargeted(boolean targeted) {
        this.targeted = targeted;
    }

    public boolean getIgnored() {
        return ignored;
    }

    public void setAlgaeIgnored(boolean ignored) {
        this.ignored = ignored;
    }
}
