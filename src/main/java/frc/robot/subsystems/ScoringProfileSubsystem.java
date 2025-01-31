// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class ScoringProfileSubsystem extends SubsystemBase {
  private int branch = 9;
  private FieldConstants.ReefHeight reefHeight  = FieldConstants.ReefHeight.L1;

  private Pose2d selectedBranchPose = new Pose2d();

  /** Creates a new ExampleSubsystem. */
  public ScoringProfileSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for (int i = 0; i < 12; i++) {
      if(DriverStation.getStickButton(1, i+1)) {
        branch = i; 
      }
    }

    for(int j=12; j<17; j++) {
      if(DriverStation.getStickButton(1, j+1)) {
        reefHeight = FieldConstants.ReefHeight.values()[j-12];
      }
    }

    selectedBranchPose = AllianceFlipUtil.apply(FieldConstants.Reef.branchPositions.get(branch).get(reefHeight).toPose2d());

    // DogLog.log("ScoringProfileSubSystem/Selected branch", branch);
    // DogLog.log("ScoringProfileSubSystem/Selected level", reefHeight);
    // DogLog.log("ScoringProfileSubSystem/Selected Pose", selectedBranchPose);
    // DogLog.log("ScoringProfileSubSystem/Robot Pose", AllianceFlipUtil.apply(selectedBranchPose.plus(new Transform2d(Units.inchesToMeters(36), 0, new Rotation2d(Math.toRadians(180))))));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public int getBranch() {
    return branch;
  }
  public FieldConstants.ReefHeight getLevel() {
    return reefHeight;
  }
  public void setBranch(char branch) {
    this.branch = branch;
  }
  public void setLevel(FieldConstants.ReefHeight level) {
    this.reefHeight = level;
  } 
  public Pose2d getSelectedBranchPose() {
    return selectedBranchPose;
  }

  public Pose2d getRobotPoseForSelectedBranch() {
    return selectedBranchPose.plus(new Transform2d(Units.inchesToMeters(36), 0, new Rotation2d(Math.toRadians(180))));
  }

}
