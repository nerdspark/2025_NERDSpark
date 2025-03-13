// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.jar.Attributes.Name;
import java.util.Map;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveToCoral;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.CoralObject;
import frc.robot.util.ArmPoint;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommandAngles;
import frc.robot.commands.ArmCommandClimb;
import frc.robot.commands.ArmCommandGripper;
import frc.robot.commands.ArmCommandGripperAutoClose;
import frc.robot.commands.ArmCommandGripperAutoCloseNeutralOpen;
import frc.robot.commands.ArmCommandPathToPoint;
import frc.robot.commands.ArmCommandWrist;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommandPickup;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.OpenGripperCommand;
import frc.robot.commands.SetStowing;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

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

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick buttonBoard = new Joystick(1);

    public Arm arm;
    private Gripper gripper;
    private Intake intake;
    private LimelightSubsystem limelightSubsystem;
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    // private Trigger armFinishedMoving = new Trigger(() -> arm.finishedMoving);



    public final CommandSwerveDrivetrain drivetrain;



    public final Vision vision = new Vision(Constants.Vision.kCameraNameFront, Constants.Vision.kRobotToCamFront);
    public final PoseEstimatorSubsystem poseEstimatorSubsystem;// = new PoseEstimatorSubsystem(drivetrain);
    

    // public final ScoringProfileSubsystem scoringSubsystem = new ScoringProfileSubsystem();

    public final ScoringProfileSubsystem scoringSubsystem;


  // private final LEDSubsytem m_LedSubsystem = new LEDSubsytem();
  // private Climb climb = new Climb();
  private Trigger armFinishedMoving = new Trigger(() -> arm.finishedMoving);
  private Trigger hasCoral = new Trigger(() -> intake.hasCoral());
  // private Trigger driveTrainFinishedMoving = new Trigger(() -> poseEstimatorSubsystem.getCurrentPose().getTranslation()
  // .getDistance(scoringSubsystem.getSelectedBranchPose().getTranslation()) < 1 || poseEstimatorSubsystem.getCurrentPose().getTranslation()
  // .getDistance((scoringSubsystem.getSelectedCoralStationPose().getTranslation()))<1);
  private Trigger driveTrainFinishedMoving = new Trigger(() -> true);
  
  /* Path follower */
  private SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrain = TunerConstants.createDrivetrain();
    poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);
    scoringSubsystem = new ScoringProfileSubsystem();
    arm = new Arm();
    gripper = new Gripper();
    intake = new Intake();

    configureNamedCommands();


    // SignalLogger.setPath("/media/sda1/armLog");
    // SignalLogger.start();
    configureBindings();
    configureDefaultCommands();
    
    // drivetrain.resetPose(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L0).plus(new Transform2d(0.1,0.1,new Rotation2d())));
    configureAutoChooser();
    if (Constants.Vision.USE_LIMELIGHT) {
      configureLimelight();
    }

  }
  private void configureLimelight() {
    limelightSubsystem = new LimelightSubsystem(drivetrain);
    joystick.y().toggleOnTrue(new DriveToCoral(drivetrain, () -> limelightSubsystem.coralArrayUpdateReturn().get(0).getPose()));
  }
  
  private void configureNamedCommands(){
    NamedCommands.registerCommand("gripperToGroundIntake", new ArmCommandPathToPoint(arm, () -> 14).alongWith(new ArmCommandGripperAutoCloseNeutralOpen(gripper)));
    NamedCommands.registerCommand("gripperOpen", new ArmCommandGripper(gripper, () -> false));
    NamedCommands.registerCommand("gripperClose", new ArmCommandGripper(gripper, () -> true));
    NamedCommands.registerCommand("armToL4", new ArmCommandPathToPoint(arm, () -> 5).alongWith(new ArmCommandGripper(gripper, () -> true)));
    NamedCommands.registerCommand("armToStow", new ArmCommandPathToPoint(arm, () -> 17));
    NamedCommands.registerCommand("intakePrepareThrow", new IntakeCommand(intake, () -> IntakeConstants.intakeThrowPreparePosition, () -> IntakeConstants.intakePassive));
    NamedCommands.registerCommand("intakeThrow", new IntakeCommand(intake, ()-> IntakeConstants.intakeThrowPosition, () -> IntakeConstants.intakePassive)
    .withTimeout(0.45)
    .andThen(new IntakeCommand(intake, ()-> IntakeConstants.intakeThrowPosition, () -> IntakeConstants.intakeThrowPower))
    .withTimeout(0.35)
      .andThen(new IntakeCommand(intake, ()-> IntakeConstants.home, () -> 0.0)));
    NamedCommands.registerCommand("test", new InstantCommand(()-> System.out.println("test")));
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(xLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY()) * MaxSpeed))
          .withVelocityY(yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed))
          .withRotationalRate(zLimiter.calculate(-joystick.getLeftX() * MaxAngularRate))
        )
        );
    
    
    //joystick.y().toggleOnTrue(new DriveToCoral(drivetrain, () -> new Pose2d(2.0, 2.0, new Rotation2d(0))));



    arm.setDefaultCommand(new ArmCommandPathToPoint(arm, () -> (arm.stowing ? 6 : 7)));

    gripper.setDefaultCommand(new ArmCommandGripperAutoClose(gripper));

    intake.setDefaultCommand(new IntakeCommandPickup(intake, () -> IntakeConstants.home, () -> 0.0));

    // climb.setDefaultCommand(new ClimbCommand(climb, () -> false));
    // m_LedSubsystem.setDefaultCommand(
    //  new LEDCommand(m_LedSubsystem, armFinishedMoving, driveTrainFinishedMoving, hasCoral)
    // );

  }


  private void configureBindings() {





    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
    //   point.withModuleDirection(new Rotation2d(-joystick.getRightY(), -joystick.getRightX()))
    // ));



    joystick.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // drivetrain.registerTelemetry(logger::telemeterize);
    // drivetrain.applyRequest(new SwerveControllerCommand(null, null, null, null, null, null));
    // joystick.a().onTrue(new ArmCommandPathToPoint(arm, 5));
    //     // joystick.b().onTrue(new ArmCommandPathToPoint(arm, 1));
    //     joystick.x().onTrue(new ArmCommandPathToPoint(arm, 2));
    //     joystick.y().onTrue(new ArmCommandPathToPoint(arm, 4));
    //     joystick.rightBumper().onTrue(new ArmCommandPathToPoint(arm, 0));
    //     gripper.setDefaultCommand(new ArmCommandGripperAutoClose(gripper));
    //     joystick.start().onTrue(Commands.runOnce(SignalLogger::stop));
    //     armFinishedMoving.onTrue(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(0.0f, 0.0f, 1.0f))));
    //     armFinishedMoving.onFalse(m_LedSubsystem.runPattern(LEDPattern.solid(new Color(1.0f, 0.0f, 0.0f))));
    //     //joystick.leftBumper().whileTrue(new ArmCommandWrist(arm, () -> WristTestAngles.testWristFlipAngle, () -> WristTestAngles.testWristTwistAngle));
    //     // joystick.a().onTrue(new ArmCommandWrist(arm, () -> WristTestAngles.testWristFlipAngle, () -> WristTestAngles.testWristTwistAngle));
    //     //drivetrain.registerTelemetry(logger::telemeterize);
    //     joystick.rightTrigger().onTrue(new ArmCommandGripper(gripper, () -> true));
    //     joystick.leftTrigger().onFalse(new ArmCommandGripper(gripper, () -> false));
    // drivetrain.registerTelemetry(logger::telemeterize);
    //     joystick.start().onTrue(Commands.runOnce(SignalLogger::stop));
        //drivetrain.registerTelemetry(logger::telemeterize);
        // joystick.rightBumper().onTrue(new ArmCommandGripper(gripper, () -> true));
        // joystick.rightTrigger().onFalse(new ArmCommandGripper(gripper, () -> false));
        // joystick.a().onTrue(new IntakeCommand(intake, () -> 0.34));


        

        //joystick.back().onTrue(new ArmCommandPathToPoint(arm, () -> 7));
        //joystick.y().onTrue(new ClimbCommand(m_ClimbSubsystem, () -> true));
        //joystick.y().onFalse(new ClimbCommand(m_ClimbSubsystem, () -> false));
        // joystick.back().onTrue(new ArmCommandPathToPoint(arm, () -> 7));
        // joystick.y().onTrue(new ClimbCommand(climb, () -> true));
        // joystick.back().whileTrue(new ClimbCommand(climb, () -> false));

        // joystick.x().onTrue(new ArmCommandPathToPoint(arm, () -> 9).alongWith(new IntakeCommand(intake, () -> IntakeConstants.climb, () -> 0.0)));
        // joystick.b().whileTrue(new ArmCommandClimb(arm, ArmConstants.shoulderPowerClimb).alongWith(new IntakeCommand(intake, () -> IntakeConstants.climb, () -> 0.0)));
        

    /* Manually start logging with left bumper before running any tests,
     * and stop logging with right bumper after we're done with ALL tests.
     * This isn't necessary but is convenient to reduce the size of the hoot file */
    SignalLogger.setPath("/media/sda1/ctre-logs/");
    // joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    

    // joystick.y().whileTrue(Autos.getAutoDriveCommandXY( drivetrain,
    // () -> drivetrain.getState().Pose, 
    // () -> scoringSubsystem.getRobotPoseForSelectedBranch(),
    // () -> scoringSubsystem.getLevel()));
  

    // // joystick.leftBumper().onTrue(new DriveToPose(drivetrain,
    // // () -> scoringSubsystem.getRobotPoseForSelectedBranch()
    // // ).until(() -> joystick.rightBumper().getAsBoolean()));

    // joystick.rightTrigger().onFalse(Autos.getDropReefOffCommand(arm, gripper, () -> scoringSubsystem.getArmReefTarget()));
    // joystick.rightTrigger().whileTrue(new ArmCommandPathToPoint(arm, () -> scoringSubsystem.getArmReefTarget()));
    joystick.rightTrigger().whileTrue(Autos.getAutoDriveCommandReef(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedBranch(),
    ()->scoringSubsystem.getLevel(),
    ()->scoringSubsystem.getIsBackwards(),
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));

    joystick.leftBumper().whileTrue(new ArmCommandPathToPoint(arm, () -> scoringSubsystem.getArmSubstationTarget()));
    joystick.leftBumper().whileTrue(Autos.getAutoDriveCommandStation(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedCoralStation(),
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));

    joystick.a().whileTrue(Autos.getAutoDriveCommandXY(drivetrain, () -> drivetrain.getState().Pose, () -> new Pose2d(), () -> ReefLevel.L1, () -> false));

    // joystick.rightStick().whileTrue(new ArmCommandPathToPoint(arm, () -> 12).alongWith(new ArmCommandGripperAutoCloseNeutralOpen(gripper)));
    // joystick.x().whileTrue(new ArmCommandPathToPoint(arm, () -> 13).alongWith(new ArmCommandGripperAutoCloseNeutralOpen(gripper)));
    // joystick.y().whileTrue(new ArmCommandPathToPoint(arm, () -> 14).alongWith(new ArmCommandGripperAutoCloseNeutralOpen(gripper)));
    joystick.povDown().onTrue(new ArmCommandGripper(gripper, () -> false));
    joystick.povUp().onTrue(new ArmCommandGripper(gripper, () -> true));


    joystick.rightBumper().whileTrue(Autos.getTransferCommand(arm, intake, gripper));
        
    joystick.leftTrigger().whileTrue((new IntakeCommandPickup(intake, () -> IntakeConstants.deploy, () -> IntakeConstants.intakePowerRollers)));
      

    joystick.start().whileTrue(new SetStowing(arm, false)); 
    joystick.back().whileTrue(new SetStowing(arm, true)); 

    // joystick.x().onTrue(new ArmCommandPathToPoint(arm, () -> 12));
    // joystick.a().onTrue(new ArmCommandPathToPoint(arm, () -> scoringSubsystem.getIsBackwards() ? 12 : 13));

    // joystick.y().onTrue(new ArmCommandPathToPoint(arm, () -> 15));
    // joystick.b().onTrue(new ArmCommandPathToPoint(arm, () -> 16));
    // joystick.y().onTrue(new DriveToPose(drivetrain,
    // () -> scoringSubsystem.getRobotPoseForSelectedBranch()
    // ).until(() -> joystick.x().getAsBoolean()));

    // joystick.povLeft().onTrue(new ArmCommandAngles(arm, () -> Units.degreesToRadians(30), () -> ArmConstants.elbowPositionClimb).alongWith(new ArmCommandWrist(arm, () -> -2.0, () -> 0.0).alongWith(new WaitCommand(0.5).andThen(new IntakeCommand(intake, () -> IntakeConstants.climb, () -> 0.0)))));
    // joystick.a().onTrue(new ArmCommandClimb(arm, ArmConstants.shoulderPowerClimb, ArmConstants.elbowPositionClimb).alongWith(new IntakeCommand(intake, () -> IntakeConstants.deploy, () -> 0.0)));
 
  }

  private void configureAutoChooser() {
    // DataLogManager.log("Configuring auto chooser");
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    // return new Command() {};
    return autoChooser.getSelected();

  }
}