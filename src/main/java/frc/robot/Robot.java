// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import org.ironmaple.simulation.SimulatedArena;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final Arm m_arm = new Arm();
    private final Joystick m_joystick = new Joystick(OperatorConstants.kJoystickPort);

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
        // DogLog.setOptions(new DogLogOptions()
        //         .withLogExtras(true)
        //         .withCaptureDs(true)
        //         .withNtPublish(true)
        //         .withCaptureNt(true));
        // DogLog.setPdh(new PowerDistribution());
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
    m_robotContainer.arm.resetOffsets();
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
        // DogLog.log("Simulation/CoralPoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        // DogLog.log("Simulation/AlgaePoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        m_arm.simulationPeriodic();
    }

    @Override
    public void close() {
        super.close();
    }
}

