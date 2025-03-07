// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class CoralObject {
    private Pose2d pose;
    private double hb;
    private double distance;
    
    public CoralObject(Pose2d pose, double hb, double distance) {
        this.pose = pose;
        this.hb = hb;
        this.distance = distance;
    }

    public double calcDistance(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double distance = Math.sqrt(x * x + y * y);
        return distance;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setCoralPose(Pose2d pose) {
        this.pose = pose;
    }

    public double getHB() {
        return hb;
    }

    public void setCoralHB(double hb) {
        this.hb = hb;
    }

    public double getDistance() {
        return distance;
    }

    public void setCoralDistance(double distance) {
        this.distance = distance;
    }
}
