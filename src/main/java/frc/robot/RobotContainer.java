// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Inventory;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.PIDToPosition;
import frc.robot.Telemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public static final Inventory inventory = new Inventory();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final PIDToPosition PID = new PIDToPosition();
    private final CommandXboxController joystick = new CommandXboxController(0);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
      drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        //DriverStation.getAlliance().get().equals(Alliance.Blue) ? AutoDriveConstants.blueConstants() : AutoDriveConstants.redConstants;
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
        joystick.a().whileTrue(new InstantCommand(() -> inventory.setAstatus(true)));// AutoDrive Command
        Trigger AStatus = new Trigger(() -> (inventory.getAstatus()&&PID.InsideRange(() -> poseEstimatorSubsystem.getCurrentPose())));
        AStatus.whileTrue(new DriveToPoseCommand(drivetrain, () -> poseEstimatorSubsystem.getCurrentPose(), 
        () -> AutoDrive(() -> poseEstimatorSubsystem.getCurrentPose()), ()->drivetrain.getState().Pose.getRotation()).until(() -> joystick.y().getAsBoolean()));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
  }
  public Pose2d AutoDrive(Supplier<Pose2d> robotPose){
        boolean inventoryStatuses = inventory.getAstatus();
        Translation2d BestVector = PID.ChooseVector(robotPose.get(), inventoryStatuses);
        SmartDashboard.putNumber("positionsX", PID.ChooseVector(robotPose.get(), inventoryStatuses).getX());
        boolean deadzoneA = robotPose.get().getTranslation().getDistance(AutoDriveConstants.position1) < AutoDriveConstants.zone;
        boolean deadzoneB = robotPose.get().getTranslation().getDistance(AutoDriveConstants.position2) < AutoDriveConstants.zone;
        boolean deadzoneC = robotPose.get().getTranslation().getDistance(AutoDriveConstants.position3) < AutoDriveConstants.zone;
        if (deadzoneA || deadzoneB || deadzoneC == true){
            inventory.setAstatus(false);
        }
        return new Pose2d(BestVector.getX(), BestVector.getY(), new Rotation2d(0.0));
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
