// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmMap;
import frc.robot.subsystems.Arm;
import frc.robot.util.ArmPath;
import frc.robot.util.ArmPathplannerUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommandFollowPath extends Command {
  private Arm arm;
    private ArmPath path;
    private Supplier<Boolean> inBend;
    /** Creates a new ArmCommand. */
    public ArmCommandFollowPath(Arm arm, ArmPath path, Supplier<Boolean> inBend) {
        this.arm = arm;
        this.path = path;
        this.inBend = inBend;
        addRequirements(arm);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//TODO: add wrist interpolation or other way to time wrist movement
    SmartDashboard.putBoolean("Check", false);
    if (ArmPathplannerUtil.CheckArmPosition(path.getTranslations(), arm.getArmPosition())){
      SmartDashboard.putBoolean("Check", true);
      arm.setArmPosition(path.getTranslations().get(path.getTranslations().size()-1), inBend.get());
    }else{
      Rotation2d direction = ArmPathplannerUtil.ArmPathChooser(path.getTranslations(), arm.getArmPosition());
      arm.setVelocity(new Translation2d(direction.getCos(), direction.getSin()).times(ArmMap.velocity));
      if ((path.getTranslations().get((path.getTranslations().size()-1)/4)).getDistance(arm.getArmPosition()) < 6.0){
        arm.setWristFlipPosition(path.points.get(path.points.size()-1).wristFlip);
        arm.setWristTwistPosition(path.points.get(path.points.size()-1).wristTwist);
      }
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
