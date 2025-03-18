// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commandSequences.ArmActions;
import frc.robot.commands.DriveToCoral;
import frc.robot.commands.GripperCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick buttonBoard = new Joystick(1);

    public Arm arm;
    private Gripper gripper;
    private Climb climb;
    private LimelightSubsystem limelightSubsystem;
    // private final Telemetry logger = new Telemetry(MaxSpeed);

    // private Trigger armFinishedMoving = new Trigger(() -> arm.finishedMoving);



    public final CommandSwerveDrivetrain drivetrain;



    public final Vision vision = new Vision(Constants.Vision.kCameraNameFront, Constants.Vision.kRobotToCamFront);
    public final PoseEstimatorSubsystem poseEstimatorSubsystem;// = new PoseEstimatorSubsystem(drivetrain);
    

    // public final ScoringProfileSubsystem scoringSubsystem = new ScoringProfileSubsystem();

    public final ScoringProfileSubsystem scoringSubsystem;


  // private final LEDSubsytem m_LedSubsystem = new LEDSubsytem();
  // private Trigger driveTrainFinishedMoving = new Trigger(() -> poseEstimatorSubsystem.getCurrentPose().getTranslation()
  // .getDistance(scoringSubsystem.getSelectedBranchPose().getTranslation()) < 1 || poseEstimatorSubsystem.getCurrentPose().getTranslation()
  // .getDistance((scoringSubsystem.getSelectedCoralStationPose().getTranslation()))<1);
  
  /* Path follower */
  private SendableChooser<Command> autoChooser;
  // private Command armDefaultCommand = new ArmDefaultCommand(arm, () -> (arm.stowing ? 6 : 7));
  // private Command gripperDefaultCommand = new ArmCommandGripperAutoClose(gripper, () -> !arm.stowing, () -> arm.stowing);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrain = TunerConstants.createDrivetrain();
    poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);
    scoringSubsystem = new ScoringProfileSubsystem();
    arm = new Arm();
    gripper = new Gripper();

    configureNamedCommands();


    // SignalLogger.setPath("/media/sda1/armLog");
    // SignalLogger.start();
    configureDefaultCommands();

    configureBindings();
    
    // drivetrain.resetPose(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L0).plus(new Transform2d(0.1,0.1,new Rotation2d())));
    configureAutoChooser();
    if (Constants.Vision.USE_LIMELIGHT) {
      configureLimelight();
    }

    // configureClimb();
  }
  private void configureLimelight() {
    limelightSubsystem = new LimelightSubsystem(drivetrain);
    joystick.y().whileTrue(new DriveToCoral(drivetrain, () -> limelightSubsystem.coralArrayUpdateReturn().get(0).getPose()));
  }
  
  private void configureNamedCommands(){
    // NamedCommands.registerCommand("gripperToGroundIntake", new WaitCommand(1.0).andThen(new ArmCommandGripperGroundPickup(gripper)).raceWith((new ArmCommandPathToPoint(arm, () -> 14))).andThen(new WaitCommand(0.2)).andThen(new ArmCommandPathToPoint(arm, () -> 18)));
    // NamedCommands.registerCommand("gripperOpen", Autos.getDunkDropCommand(arm, gripper, () -> 18));//new ArmCommandGripper(gripper, () -> false).alongWith(new ArmCommandPathToPoint(arm, () -> 18)));
    // NamedCommands.registerCommand("gripperOpenThenGroundIntake", new ArmCommandGripper(gripper, () -> false).withTimeout(0.25).andThen((new WaitCommand(1.0).andThen(new ArmCommandGripperGroundPickup(gripper))).raceWith((new ArmCommandPathToPoint(arm, () -> 14))).andThen(new WaitCommand(0.2)).andThen(new ArmCommandPathToPoint(arm, () -> 18))));
    // NamedCommands.registerCommand("armToStow", new ArmCommandPathToPoint(arm, () -> 17));
    // NamedCommands.registerCommand("armToHome", new ArmCommandPathToPoint(arm, () -> 7));
    // NamedCommands.registerCommand("armToL4", new ArmCommandPathToPoint(arm, () -> 18));
    // NamedCommands.registerCommand("intakeThrow", new IntakeCommandPower(intake, ()-> IntakeConstants.intakeThrowDeployPower, () -> IntakeConstants.intakePassive).until(() -> intake.getIntakeDeployPosition() < IntakeConstants.intakeThrowPosition)
    // .andThen(new IntakeCommandPower(intake, ()-> IntakeConstants.intakeThrowDeployPower, () -> IntakeConstants.intakeThrowPower)
    //   .withTimeout(0.05))
    //   .andThen(new IntakeCommand(intake, ()-> IntakeConstants.home, () -> IntakeConstants.intakeThrowPower).withTimeout(1.0).andThen(() ->intake.stopIntake())));
    // NamedCommands.registerCommand("test", new InstantCommand(()-> System.out.println("test")));


    // // coach elmer didn't want us to break the arm
    // NamedCommands.registerCommand("armToStowPrint", new InstantCommand(()-> System.out.println("armToStowPrint")));
    // NamedCommands.registerCommand("intakePrepareThrowPrint", new InstantCommand(()-> System.out.println("intakePrepareThrowPrint")));
    // NamedCommands.registerCommand("intakeThrowPrint", new InstantCommand(()-> System.out.println("intakeThrowPrint")));
    // NamedCommands.registerCommand("gripperToGroundIntakePrint", new InstantCommand(()-> System.out.println("gripperToGroundIntakePrint")));
    // NamedCommands.registerCommand("armToL4Print", new InstantCommand(()-> System.out.println("armToL4Print")));

  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(xLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY()) * MaxSpeed))
          .withVelocityY(yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed))
          .withRotationalRate(zLimiter.calculate(-joystick.getLeftX() * MaxAngularRate))
        )
        );
    
    



    arm.setDefaultCommand(new ArmCommand(arm, () -> 0));

    gripper.setDefaultCommand(new GripperCommand(gripper, 1.0));



    // climb.setDefaultCommand(new ClimbCommand(climb, () -> false));
    // m_LedSubsystem.setDefaultCommand(
    //  new LEDCommand(m_LedSubsystem, armFinishedMoving, driveTrainFinishedMoving, hasCoral)
    // );

  }


  private void configureBindings() {
    joystick.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    joystick.leftBumper().onTrue(ArmActions.moveToCoralReef(arm, () -> scoringSubsystem.getArmReefTarget()));

  }
  private void configureClimb() {
    climb = new Climb();
    // joystick.povUp().onTrue(ClimbSequence.prepareToClimb(arm, intake, climb));
    // joystick.povRight().onTrue(ClimbSequence.closeHooks(arm, intake, climb));
    // joystick.povDown().onTrue(ClimbSequence.climbDown(arm, intake, climb));
    // joystick.povLeft().onTrue(ClimbSequence.latch(arm, intake, climb));
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