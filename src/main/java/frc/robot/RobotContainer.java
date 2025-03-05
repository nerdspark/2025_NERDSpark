// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import java.util.Map;

import java.util.Map;

import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommandGripper;
import frc.robot.commands.ArmCommandGripperAutoClose;
import frc.robot.commands.ArmCommandPathToPoint;
import frc.robot.commands.ArmCommandWrist;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommandPickup;
import frc.robot.commands.OpenGripperCommand;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;

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



    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();



    public final Vision vision = new Vision(Constants.Vision.kCameraNameFront, Constants.Vision.kRobotToCamFront);
    public final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);

    public final ScoringProfileSubsystem scoringSubsystem = new ScoringProfileSubsystem();


  // private final LEDSubsytem m_LedSubsystem = new LEDSubsytem();
  private Climb climb = new Climb();
  private Trigger armFinishedMoving = new Trigger(() -> arm.finishedMoving);
  private Trigger drivetrainFinishedMoving = new Trigger (() -> poseEstimatorSubsystem.getCurrentPose().getTranslation()
  .getDistance(scoringSubsystem.getSelectedBranchPose().getTranslation()) < 1 || poseEstimatorSubsystem.getCurrentPose().getTranslation()
  .getDistance((scoringSubsystem.getSelectedCoralStationPose().getTranslation()))<1);


  /* Path follower */
  private final SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // scoringSubsystem = new ScoringProfileSubsystem();

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    arm = new Arm();
    gripper = new Gripper();
    intake = new Intake();
    SignalLogger.setPath("/media/sda1/armLog");
    SignalLogger.start();
    configureBindings();
    configureDefaultCommands();
    configureLEDs();
    drivetrain.resetPose(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L0).plus(new Transform2d(0.1,0.1,new Rotation2d())));

  }
  
  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(xLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY()) * MaxSpeed))
          .withVelocityY(yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed))
          .withRotationalRate(zLimiter.calculate(-joystick.getLeftX() * MaxAngularRate))
        )
        );



    arm.setDefaultCommand(new ArmCommandPathToPoint(arm, () -> 6));

    gripper.setDefaultCommand(new ArmCommandGripperAutoClose(gripper));

    intake.setDefaultCommand(new IntakeCommandPickup(intake, () -> IntakeConstants.home, () -> 0.0));

    climb.setDefaultCommand(new ClimbCommand(climb, () -> false));

  }


  private void configureBindings() {





    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
    //   point.withModuleDirection(new Rotation2d(-joystick.getRightY(), -joystick.getRightX()))
    // ));



    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // drivetrain.registerTelemetry(logger::telemeterize);
    //     joystick.start().onTrue(Commands.runOnce(SignalLogger::stop));
        //drivetrain.registerTelemetry(logger::telemeterize);
        joystick.rightBumper().onTrue(new ArmCommandGripper(gripper, () -> true));
        joystick.rightTrigger().onFalse(new ArmCommandGripper(gripper, () -> false));
        // joystick.a().onTrue(new IntakeCommand(intake, () -> 0.34));


        joystick.leftBumper().whileTrue(Autos.getTransferCommand(arm, intake, gripper));
        
        joystick.leftTrigger().whileTrue((new IntakeCommandPickup(intake, () -> IntakeConstants.deploy, () -> IntakeConstants.intakePowerRollers)));
          
        joystick.back().onTrue(new ArmCommandPathToPoint(arm, () -> 7));
        joystick.y().whileTrue(new ClimbCommand(climb, () -> true));
        // joystick.y().onFalse(new ClimbCommand(m_ClimbSubsystem, () -> false));

        joystick.x().onTrue(new ArmCommand(arm, () -> Constants.ArmSetpoints.armSetPoints[9]));
        joystick.b().onTrue(new ArmCommand(arm, () -> Constants.ArmSetpoints.armSetPoints[10]));
        

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


    // joystick.y().whileTrue(Autos.getAutoDriveCommandXY( drivetrain,
    // () -> drivetrain.getState().Pose, 
    // () -> scoringSubsystem.getRobotPoseForSelectedBranch(),
    // () -> scoringSubsystem.getLevel()));
  

    // // joystick.leftBumper().onTrue(new DriveToPose(drivetrain,
    // // () -> scoringSubsystem.getRobotPoseForSelectedBranch()
    // // ).until(() -> joystick.rightBumper().getAsBoolean()));

    joystick.povDown().whileTrue(Autos.getAutoDriveCommandReef(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedBranch(),
    ()->scoringSubsystem.getLevel(),
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()).alongWith(new ArmCommandPathToPoint(arm, () -> (scoringSubsystem.getLevel().level))));

    joystick.povUp().whileTrue(Autos.getAutoDriveCommandStation(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedCoralStation(),
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));

    joystick.a().whileTrue(new ArmCommandPathToPoint(arm, () -> (scoringSubsystem.getLevel().level)));

    // joystick.y().onTrue(new DriveToPose(drivetrain,
    // () -> scoringSubsystem.getRobotPoseForSelectedBranch()
    // ).until(() -> joystick.x().getAsBoolean()));
 
  }
  private void configureLEDs() {
    LEDPattern greenPattern = LEDPattern.solid(new Color(1.0f, 0.0f, 0.0f));
    LEDPattern bluePattern = LEDPattern.solid(new Color(0.0f, 0.0f, 1.0f));
      
    
    // armFinishedMoving.onTrue(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(0.0f, 0.0f, 1.0f))));
    // armFinishedMoving.onFalse(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(1.0f, 0.0f, 0.0f))));

    // drivetrainFinishedMoving.onTrue(m_LedSubsystem.runPattern(greenPattern.blink(Seconds.of(0.5))));
    // drivetrainFinishedMoving.onFalse(m_LedSubsystem.runPattern(bluePattern.blink(Seconds.of(0.5))));

    // Color step1 = new Color();
    // Color step2 = new Color();
    
    //   if(armFinishedMoving.getAsBoolean()) {
    //     step1 = new Color(0.0f, 0.0f, 1.0f); // blue
    //   } else {
    //     step1 = new Color(1.0f, 0.0f, 0.0f); // green
    //   }
    //   if(drivetrainFinishedMoving.getAsBoolean()) {
    //     step2 = new Color(0.0f, 1.0f, 0.0f); // red
    //   } else {
    //     step2 = new Color(1.0f, 1.0f, 0.0f); // yellow
    //   }
    //   LEDPattern steps = LEDPattern.steps(Map.of(0, step1, 0.5, step2))
    //     .scrollAtRelativeSpeed(Percent.per(Second).of(40));
    //   m_LedSubsystem.runPattern(steps);


    
    
    
    
    

  //  drivetrainFinishedMoving.onFalse(m_LedSubsystem.runPattern(LEDPattern.gradient(LED, greenPattern, bluePattern)));
    // intake status
    // error states

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