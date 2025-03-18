// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.util.AllianceFlipUtil;

public class ScoringProfileSubsystem extends SubsystemBase {

 
  private int branch = 2;
  private int level = 0;
  private FieldConstants.ReefLevel reefLevel  = FieldConstants.ReefLevel.L3;
  private FieldConstants.CoralStations coralStationSide = FieldConstants.CoralStations.LEFT;
  private Pose2d selectedBranchPose = new Pose2d();
  private Pose2d selectedCoralStationPose = new Pose2d();

  private static final int [] branchesSimon = {5,4,3,2,1,0,11,10,9,8,7,6}; // Needed as the button board is assembled in incorrect orientation
  private static final int [] branchesSayan = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0}; // Needed as the button board is assembled in incorrect orientation


  /** Creates a new ExampleSubsystem. */
  public ScoringProfileSubsystem() {  
  }

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
  public int getArmReefTarget() {
    
    return reefLevel.level;
    
  }
  public int getArmSubstationTarget() {
    return 13;
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
    // SmartDashboard.putBoolean("isbackwards", isBackwards);
    // SmartDashboard.putString("reeflevel", reefLevel.name());
    // SmartDashboard.putString("coralstationside", coralStationSide.name());
    // SmartDashboard.putNumber("branch", branch);

  if(Constants.Vision.USE_BUTTON_BOARD) {
    for (int i = 0; i < 12; i++) {
      if(DriverStation.getStickButton(1, i+1)) {
        branch = branchesSayan[i]; //Adjusting for incorrect button board orientation.
      }
    }

    for(int j=12; j<17; j++) {
      if(DriverStation.getStickButton(1, j+1)) {
        reefLevel = FieldConstants.ReefLevel.values()[j-12];
        // System.out.println("J: " + j + "; reeflevel: " + reefLevel.level);
      }
    }


    for(int k=17; k<19; k++) {
      if(DriverStation.getStickButton(1, k+1)) {
        coralStationSide = FieldConstants.CoralStations.values()[k-17];
      }
    }

    

  }
  else {
    if(DriverStation.getStickButtonPressed(0, 7)) {
      branch++;
      if(branch > 11) {
        branch = 0;
      }
    }
    if(DriverStation.getStickButtonPressed(0, 1)) {
      level++;
      reefLevel = FieldConstants.ReefLevel.fromLevel(level);
      if(level > 5) {
        level = 0;
      }
    }
          
  }
  if (Constants.Vision.DOGLOG_ENABLED){

    DogLog.log("ScoringProfileSubSystem/Selected Branch", branch);
    DogLog.log("ScoringProfileSubSystem/Selected ReefLevel", reefLevel);
    DogLog.log("ScoringProfileSubSystem/Selected CoralStation", coralStationSide);
  }
    selectedBranchPose = AllianceFlipUtil.apply(FieldConstants.Reef.branchPositions.get(branch).get(reefLevel).toPose2d());
    if(coralStationSide == FieldConstants.CoralStations.LEFT) {
      selectedCoralStationPose = AllianceFlipUtil.apply(FieldConstants.CoralStation.leftCenterFace);
    }
    else {
      selectedCoralStationPose = AllianceFlipUtil.apply(FieldConstants.CoralStation.rightCenterFace);
    }

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
  public FieldConstants.ReefLevel getLevel() {
    return reefLevel;
  }
  public void setBranch(char branch) {
    this.branch = branch;
  }
  public void setLevel(FieldConstants.ReefLevel level) {
    this.reefLevel = level;
  } 
  public Pose2d getSelectedBranchPose() {
    return selectedBranchPose;
  }
  public Pose2d getSelectedCoralStationPose() {
    return selectedCoralStationPose;
  }


  public FieldConstants.CoralStations getCoralStationSide() {
    return coralStationSide;
  }

  public void setCoralStationSide(FieldConstants.CoralStations coralStationSide) {
    this.coralStationSide = coralStationSide;
  }

  public Pose2d getRobotPoseForSelectedBranch() {
   
      return selectedBranchPose.plus(Constants.Vision.reefLevelOffsetsMap.get(reefLevel));

  }

  public Pose2d getRobotPoseForSelectedCoralStation() {
    return selectedCoralStationPose.plus(Constants.Vision.coralStationOffSetsMap.get(coralStationSide));
  }

}
