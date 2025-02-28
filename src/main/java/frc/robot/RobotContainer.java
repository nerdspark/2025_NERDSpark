// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ArmTestAngles;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;
import frc.robot.Constants.WristTestAngles;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommandAngles;
import frc.robot.commands.ArmCommandFollowPath;
import frc.robot.commands.ArmCommandGripper;
import frc.robot.commands.ArmCommandGripperAutoClose;
import frc.robot.commands.ArmCommandPathToPoint;
import frc.robot.commands.ArmCommandWrist;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.util.ArmPath;
import frc.robot.util.ArmPathplannerUtil;
import frc.robot.util.ArmPoint;
import frc.robot.util.GenPath;
import frc.robot.Telemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SlewRateLimiter xLimiter = new SlewRateLimiter(10);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(10);
  private SlewRateLimiter zLimiter = new SlewRateLimiter(25);    
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick buttonBoard = new Joystick(1);

    public Arm arm;
    private Gripper gripper;
    private Intake intake;
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private Trigger armFinishedMoving = new Trigger(() -> arm.finishedMoving);


    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    // public final  Vision vision = new Vision(Constants.Vision.kCameraName, Constants.Vision.kRobotToCam);
    public final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);

    public final ScoringProfileSubsystem scoringSubsystem = new ScoringProfileSubsystem();


  // The robot's subsystems and commands are defined here...
  private final LEDSubsytem m_LedSubsystem = new LEDSubsytem();

  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));
    arm = new Arm();
    gripper = new Gripper();
    intake = new Intake();
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



    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(xLimiter.calculate(Constants.joystickMap.get(-joystick.getRightY()) * MaxSpeed))
          .withVelocityY(yLimiter.calculate(Constants.joystickMap.get(-joystick.getRightX()) * MaxSpeed))
          .withRotationalRate(zLimiter.calculate(-joystick.getLeftX() * MaxAngularRate))
        )
    );

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() ->
      point.withModuleDirection(new Rotation2d(-joystick.getRightY(), -joystick.getRightX()))
    ));


    // reset the field-centric 
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
    // drivetrain.applyRequest(new SwerveControllerCommand(null, null, null, null, null, null));
    joystick.a().onTrue(new ArmCommandPathToPoint(arm, 5));
        // joystick.b().onTrue(new ArmCommandPathToPoint(arm, 1));
        joystick.x().onTrue(new ArmCommandPathToPoint(arm, 2));
        joystick.y().onTrue(new ArmCommandPathToPoint(arm, 4));
        joystick.rightBumper().onTrue(new ArmCommandPathToPoint(arm, 0));
        gripper.setDefaultCommand(new ArmCommandGripperAutoClose(gripper));
        joystick.start().onTrue(Commands.runOnce(SignalLogger::stop));
        armFinishedMoving.onTrue(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(0.0f, 0.0f, 1.0f))));
        armFinishedMoving.onFalse(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(1.0f, 0.0f, 0.0f))));
        //joystick.leftBumper().whileTrue(new ArmCommandWrist(arm, () -> WristTestAngles.testWristFlipAngle, () -> WristTestAngles.testWristTwistAngle));
        // joystick.a().onTrue(new ArmCommandWrist(arm, () -> WristTestAngles.testWristFlipAngle, () -> WristTestAngles.testWristTwistAngle));
        //drivetrain.registerTelemetry(logger::telemeterize);
        joystick.rightTrigger().onTrue(new ArmCommandGripper(gripper, () -> true));
        joystick.leftTrigger().onFalse(new ArmCommandGripper(gripper, () -> false));
        
        
        // joystick.b().onTrue(new ArmCommandGripper(gripper, () -> false));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    /* Manually start logging with left bumper before running any tests,
     * and stop logging with right bumper after we're done with ALL tests.
     * This isn't necessary but is convenient to reduce the size of the hoot file */
    SignalLogger.setPath("/media/sda1/ctre-logs/");
    // joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    
        /*
     * Joystick Y = quasistatic forward
     * Joystick A = quasistatic reverse
     * Joystick B = dynamic forward
     * Joystick X = dyanmic reverse
     */
    // joystick.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // joystick.a().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // joystick.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // joystick.x().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    joystick.y().whileTrue(Autos.getAutoDriveCommandXY( drivetrain,
    () -> drivetrain.getState().Pose, 
    () -> scoringSubsystem.getRobotPoseForSelectedBranch(),
    () -> scoringSubsystem.getLevel()));
  

    // joystick.leftBumper().onTrue(new DriveToPose(drivetrain,
    // () -> scoringSubsystem.getRobotPoseForSelectedBranch()
    // ).until(() -> joystick.rightBumper().getAsBoolean()));

    joystick.leftBumper().whileTrue(Autos.getAutoDriveCommandReef(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedBranch(),
    ()->scoringSubsystem.getLevel(),
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));

    joystick.rightBumper().whileTrue(Autos.getAutoDriveCommandStation(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedCoralStation(),
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));

    // joystick.y().onTrue(new DriveToPose(drivetrain,
    // () -> scoringSubsystem.getRobotPoseForSelectedBranch()
    // ).until(() -> joystick.x().getAsBoolean()));
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();

  }
}