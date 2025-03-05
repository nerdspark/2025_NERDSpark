// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.LEDSubsytem.LEDSubsystem;
import frc.robot.util.ArmPath;
import frc.robot.util.ArmPathplannerUtil;
import frc.robot.util.ArmPoint;
import frc.robot.util.GenPath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommandPathToPoint extends Command {
  private Arm arm;
    private ArmPath path;
    private IntSupplier setPoint;
    private int setPointInt;
    private double initialWristFlip, initialWristTwist;
    /** path to the specified armPoint */ // TODO: add inflection point generation and hardstop avoidance to automatic path generation
    public ArmCommandPathToPoint(Arm arm, IntSupplier setPoint) {
      this.setPoint = setPoint;
        this.arm = arm;
        addRequirements(arm);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialWristFlip = arm.getWristFlipPosition();
    initialWristTwist = arm.getWristTwistPosition();
    arm.finishedMoving = false;
    calculatePath();

  }
  public void calculatePath() {
    setPointInt = setPoint.getAsInt();
    int closestPoint = ArmPathplannerUtil.closestArmPoint(ArmSetpoints.armSetPoints, arm.getArmPosition());
    if (closestPoint != setPointInt) {
      List<ArmPoint> temp = new ArrayList<>();
      temp.add(new ArmPoint(arm.getArmPosition(), arm.getCurrentInBend()));
      temp.addAll(ArmSetpoints.intermediatePoints[closestPoint][setPointInt]);
      temp.add(ArmSetpoints.armSetPoints[setPointInt]);
      // path = new ArmPath(GenPath.generateSmoothPath(GenPath.generateInflectionPoints(temp), ArmConstants.arcRadius, ArmConstants.arcPoints));
      path = new ArmPath(GenPath.generateInflectionPoints(temp));
    } else {
      path = new ArmPath(List.of(ArmSetpoints.armSetPoints[setPointInt]));
    }
    // System.out.println(path.toStringList().toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//TODO: add wrist interpolation or other way to time wrist movement
    if (ArmPathplannerUtil.CheckArmPosition(path.getTranslations(), arm.getArmPosition())) {
      arm.finishedMoving = true;
    }
    if (setPointInt != setPoint.getAsInt()) {
      calculatePath();
      arm.finishedMoving = false;
    }
    // SmartDashboard.putBoolean("Check", false);
    if (arm.finishedMoving){
      // SmartDashboard.putBoolean("Check", true);
      arm.setArmPosition(path.getTranslations().get(path.getTranslations().size()-1), path.points.get(path.getTranslations().size() - 1).inBend);
      // LEDSubsystem.runPattern(LEDPattern.solid(new Color(0.0f, 0.0f, 1.0f)));
    } else {
      ArmPoint nextPoint = ArmPathplannerUtil.getNextPoint(path.points, arm.getArmPosition());
      Rotation2d direction = ArmPathplannerUtil.ArmPathChooser(path.getTranslations(), arm.getArmPosition());
      arm.setVelocity(new Translation2d(direction.getCos(), direction.getSin()).times(ArmConstants.velocity), nextPoint.inBend);
      // LEDSubsystem.runPattern(LEDPattern.solid(new Color(0.0f, 1.0f, 0.0f)));
    }

    int nextPointIndex = ArmPathplannerUtil.getNextPointIndex(path.points, arm.getArmPosition());
    double pathProgress = nextPointIndex/path.points.size();
    double finalWristFlip = path.points.get(path.points.size() - 1).wristFlip;
    double finalWristTwist = path.points.get(path.points.size() - 1).wristTwist;
      if (((pathProgress > 0.65) || arm.finishedMoving)){
        arm.setWristTwistPosition(finalWristTwist);
        if ((pathProgress > 0.8) || arm.finishedMoving){
          arm.setWristFlipPosition(finalWristFlip);
        }
      } else {
        arm.setWristFlipPosition((pathProgress * (finalWristFlip-initialWristFlip)) + initialWristFlip);
        arm.setWristTwistPosition((pathProgress * (finalWristTwist-initialWristTwist)) + initialWristTwist);
        // arm.stopWrist();
        
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
