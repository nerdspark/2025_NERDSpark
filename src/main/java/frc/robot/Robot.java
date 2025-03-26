// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Vision;

import org.ironmaple.simulation.SimulatedArena;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
    private final boolean kUseLimelight = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        
        FollowPathCommand.warmupCommand().schedule();

        // Pathfinding.setPathfinder(new LocalADStar());
        // PathfindingCommand.warmupCommand().schedule();

        if(!Vision.DOGLOG_ENABLED){
            DogLog.setEnabled(false);
        }else{
            DogLog.setEnabled(true);
            DogLog.setOptions(new DogLogOptions()
                .withLogExtras(true)
                .withCaptureDs(true)
                .withNtPublish(true)
                .withCaptureNt(true));
                DogLog.setPdh(new PowerDistribution());
        }

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        // if (kUseLimelight) {
        //     var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        //     if (llMeasurement != null) {
        //         m_robotContainer.drivetrain.addVisionMeasurement(
        //                 llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
        //     }
        // }
        SmartDashboard.putNumber("Match Time",DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopPeriodic() {
   
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
        // m_robotContainer.arm.setArmPosition(ArmSetpoints.home.plus(new Translation2d(4, 2)), false);

// TODO: remove after testing
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } 
  }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        DogLog.log("Simulation/CoralPoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        DogLog.log("Simulation/AlgaePoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

    }

    @Override
    public void close() {
        super.close();
    }
}

