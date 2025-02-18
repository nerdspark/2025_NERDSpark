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
import frc.robot.util.ArmPathplannerUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommandFollowPath extends Command {
  private Arm arm;
    private List<Translation2d> path;
    private Supplier<Boolean> inBend;
    /** Creates a new ArmCommand. */
    public ArmCommandFollowPath(Arm arm, List<Translation2d> path, Supplier<Boolean> inBend) {
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
  public void execute() {
    SmartDashboard.putBoolean("Check", false);
    if (ArmPathplannerUtil.CheckArmPosition(path, arm.getArmPosition())){
      SmartDashboard.putBoolean("Check", true);
      arm.setArmPosition(path.get(path.size()-1), inBend.get());
    }else{
      Rotation2d direction = ArmPathplannerUtil.ArmPathChooser(path, arm.getArmPosition());
      arm.setVelocity(new Translation2d(direction.getCos(), direction.getSin()).times(ArmMap.velocity));
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
