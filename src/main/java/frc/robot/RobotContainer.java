// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ArmTestAngles;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristTestAngles;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommandAngles;
import frc.robot.commands.ArmCommandFollowPath;
import frc.robot.commands.ArmCommandGripper;
import frc.robot.commands.ArmCommandGripperAutoClose;
import frc.robot.commands.ArmCommandPathToPoint;
import frc.robot.commands.ArmCommandWrist;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.Gripper;
import frc.robot.util.ArmPath;
import frc.robot.util.ArmPathplannerUtil;
import frc.robot.util.ArmPoint;
import frc.robot.util.GenPath;
import frc.robot.Telemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public Arm arm;
    private Gripper gripper;
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    // private final CommandXboxController joystick = new CommandXboxController(0);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final LEDSubsytem m_LedSubsystem = new LEDSubsytem();

  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    arm = new Arm();
    gripper = new Gripper();
    SignalLogger.setPath("/media/sda1/armLog");
    SignalLogger.start();
    // Configure the trigger bindings
    configureBindings();
    // drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));
    // m_LedSubsystem.setDefaultCommand(m_LedSubsystem.runPattern(LEDPattern.solid(Color.kRed)).withName("On"));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      // drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // drivetrain.applyRequest(() ->
                // drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    // .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    // .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              // )
              // m_LedSubsystem.runPattern()
        // );

      

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
            // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    joystick.x().whileTrue(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(0.0f, 0.0f, 1.0f))));
    joystick.a().whileTrue(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(1.0f, 0.0f, 0.0f))));
    joystick.y().whileTrue(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(0.0f, 1.0f, 0.0f))));
    joystick.b().whileTrue(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(1.0f, 1.0f, 1.0f))));

      //       // Drivetrain will execute this command periodically
      //       drivetrain.applyRequest(() ->
      //           drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
      //               .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
      //               .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      //       )
      //   );

      //   joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
      //   joystick.b().whileTrue(drivetrain.applyRequest(() ->
      //       point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
      //   ));

      //   // Run SysId routines when holding back/start and X/Y.
      //   // Note that each routine should be run exactly once in a single log.
      //   joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      //   joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      //   joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      //   joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

      //   // reset the field-centric heading on left bumper press
      //   joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.rightBumper().onTrue(new ArmCommand(arm, () -> ReefSetPoints.l1Reef, () -> false));
        // // joystick.x().onTrue(new ArmCommandAngles(arm, () -> ArmTestAngles.testElbowAngle, () -> ArmTestAngles.testShoulderAngle));
        // joystick.x().onTrue(new ArmCommandAngles(arm, () -> ArmTestAngles.testElbowAngle, () -> ArmTestAngles.testShoulderAngle));
        // joystick.y().onTrue(new ArmCommandFollowPath(arm, ArmSetpoints.armPaths[1][4], () -> false));
        // joystick.b().onTrue(new ArmCommandFollowPath(arm, ArmSetpoints.armPaths[0][1], () -> false));
        // joystick.b().onTrue(new ArmCommandFollowPath(arm, GenPath.generateSmoothPath(new ArmPoint(arm.getArmPosition()), () -> ArmSetpoints.intermediatePoints[ArmPathplannerUtil.closestArmPoint(ArmSetpoints.armSetPoints, () -> arm.getArmPosition())][1], ArmSetpoints.armSetPoints[1], 5, 10), () -> false));
        // joystick.y().onTrue(new ArmCommandFollowPath(arm, GenPath.generateSmoothPath(new ArmPoint(arm.getArmPosition()), () -> ArmSetpoints.intermediatePoints[ArmPathplannerUtil.closestArmPoint(ArmSetpoints.armSetPoints, () -> arm.getArmPosition())][4], ArmSetpoints.armSetPoints[4], 5, 10), () -> false));
        joystick.a().onTrue(new ArmCommandPathToPoint(arm, 5));
        // joystick.b().onTrue(new ArmCommandPathToPoint(arm, 1));
        joystick.x().onTrue(new ArmCommandPathToPoint(arm, 2));
        joystick.y().onTrue(new ArmCommandPathToPoint(arm, 4));
        joystick.rightBumper().onTrue(new ArmCommandPathToPoint(arm, 0));
        gripper.setDefaultCommand(new ArmCommandGripperAutoClose(gripper));
        joystick.start().onTrue(Commands.runOnce(SignalLogger::stop));
        //joystick.leftBumper().whileTrue(new ArmCommandWrist(arm, () -> WristTestAngles.testWristFlipAngle, () -> WristTestAngles.testWristTwistAngle));
        // joystick.a().onTrue(new ArmCommandWrist(arm, () -> WristTestAngles.testWristFlipAngle, () -> WristTestAngles.testWristTwistAngle));
        //drivetrain.registerTelemetry(logger::telemeterize);
        joystick.rightTrigger().onTrue(new ArmCommandGripper(gripper, () -> true));
        joystick.leftTrigger().onFalse(new ArmCommandGripper(gripper, () -> false));
        
        // joystick.b().onTrue(new ArmCommandGripper(gripper, () -> false));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.print("No autonomous command configured");
  }
}
