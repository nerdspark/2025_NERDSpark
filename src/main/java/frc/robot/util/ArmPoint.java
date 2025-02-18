// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmSetpoints;

/** a single setpoint for an arm position */
public class ArmPoint {
    public Translation2d position = ArmSetpoints.home;
    public boolean inBend = false;
    public double wristFlip, wristTwist = 0;
    public ArmPoint(Translation2d point, boolean inBend, double wristFlip, double wristTwist) {
        this.position = point;
        this.inBend = inBend;
        this.wristFlip = wristFlip;
        this.wristTwist = wristTwist;
    }
    public ArmPoint(Translation2d point, boolean inBend) {
        this.position = point;
        this.inBend = inBend;
    }
    public ArmPoint(Translation2d point, double wristFlip, double wristTwist) {
        this.position = point;
        this.wristFlip = wristFlip;
        this.wristTwist = wristTwist;
    }
    public ArmPoint(Translation2d point) {
        this.position = point;
    }
}
