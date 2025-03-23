// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import java.rmi.dgc.Lease;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commandSequences.ArmActions;
import frc.robot.commandSequences.Autos;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmCommandPathToPoint;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LEDSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Bucket;
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
  // private final LEDSubsytem m_LedSubsystem = new LEDSubsytem();
  
  public Arm arm;
  private Gripper gripper;
  private LEDSubsytem LEDs;
  private Bucket bucket;
  private Climb climb;

  public static BooleanSupplier autoBucketEnabled = () -> true;

  private BooleanSupplier driveTrainFinishedMoving = () -> false;
  private BooleanSupplier gripperHasGamePiece = () -> false;
  private BooleanSupplier bucketHasCoral = () -> false;
    private Trigger bucketHasCoralTrigger = new Trigger(bucketHasCoral);
    private Trigger driveTrainFinishedMovingTrigger = new Trigger(driveTrainFinishedMoving);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
             .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    // private Trigger armFinishedMoving = new Trigger(() -> arm.finishedMoving);



    public final CommandSwerveDrivetrain drivetrain;



    public final PoseEstimatorSubsystem poseEstimatorSubsystem;// = new PoseEstimatorSubsystem(drivetrain);
    

    // public final ScoringProfileSubsystem scoringSubsystem = new ScoringProfileSubsystem();

    public final ScoringProfileSubsystem scoringSubsystem;



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
    bucket = new Bucket();
    climb = new Climb();

    configureTriggers();
    configureNamedCommands();


    // SignalLogger.setPath("/media/sda1/armLog");
    // SignalLogger.start();
    configureDefaultCommands();

    configureBindings();
    
    // drivetrain.resetPose(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L0).plus(new Transform2d(0.1,0.1,new Rotation2d())));
    configureAutoChooser();
    configureLEDs();

  }
  
  private void configureNamedCommands(){
    NamedCommands.registerCommand("armToHome", arm.goToHome());
    NamedCommands.registerCommand("grabFromFunnel", ArmActions.grabFromFunnel(arm, gripper));
    NamedCommands.registerCommand("gripperToGroundIntake", ArmActions.groundIntake(arm, gripper));
    NamedCommands.registerCommand("armToL4", ArmActions.armToCoralReef(arm, gripper, () -> 4));
    NamedCommands.registerCommand("dropOffCoral", ArmActions.dunkDropCoral(arm, gripper, () -> 4));//new ArmCommandGripper(gripper, () -> false).alongWith(new ArmCommandPathToPoint(arm, () -> 18)));
    NamedCommands.registerCommand("waitUntilBucketHasCoral", new WaitUntilCommand(bucketHasCoral));
    // NamedCommands.registerCommand("gripperOpenThenGroundIntake", new ArmCommandGripper(gripper, () -> false).withTimeout(0.25).andThen((new WaitCommand(1.0).andThen(new ArmCommandGripperGroundPickup(gripper))).raceWith((new ArmCommandPathToPoint(arm, () -> 14))).andThen(new WaitCommand(0.2)).andThen(new ArmCommandPathToPoint(arm, () -> 18))));
    // NamedCommands.registerCommand("armToStow", new ArmCommandPathToPoint(arm, () -> 17));
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
    
    



    // arm.setDefaultCommand(new ArmCommand(arm, () -> 0));

    // gripper.setDefaultCommand(new GripperCommand(gripper));



  }
  private void configureTriggers() {
    bucketHasCoral = () -> bucket.getDetected();
    driveTrainFinishedMoving = () -> poseEstimatorSubsystem.getCurrentPose().getTranslation()
    .getDistance(scoringSubsystem.getSelectedBranchPose().getTranslation()) < 1;
    //  || poseEstimatorSubsystem.getCurrentPose().getTranslation().getDistance((scoringSubsystem.getSelectedCoralStationPose().getTranslation()))<1;
    gripperHasGamePiece = () -> Bucket.gripperHasGamePiece;
    bucketHasCoralTrigger = new Trigger(bucketHasCoral).and(() -> DriverStation.isTeleop()).and(() -> !Bucket.gripperHasGamePiece).and(() -> (arm.getArmPosition().getDistance(ArmSetpoints.home) < 5));
    // bucketHasCoralTrigger = new Trigger(bucketHasCoral);
    
  }


  private void configureBindings() {
    // if(bucketHasCoral.get()) {
    //   new InstantCommand(() -> joystick.setRumble(RumbleType.kBothRumble, 1));}
    // else {
    //   new InstantCommand(() -> joystick.setRumble(RumbleType.kBothRumble, 0));}


    joystick.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // * arm testing *
    // joystick.leftBumper().onTrue(ArmActions.armToCoralReef(arm, () -> scoringSubsystem.getArmReefTarget()));
    // joystick.a().onTrue(ArmActions.grabFromFunnel(arm, gripper));
    // joystick.rightBumper().onTrue(ArmActions.groundIntake(arm, gripper));
    // joystick.leftTrigger().onTrue(ArmActions.dunkDropCoral(arm, gripper, () -> scoringSubsystem.getArmReefTarget()));
    // joystick.x().onTrue(ArmActions.removeAlgae(arm, gripper, () -> ((scoringSubsystem.getBranch() / 2 % 2) == 0)));
    // joystick.y().onTrue(ArmActions.shootAlgaeBarge(gripper));
    // joystick.rightTrigger().onTrue(ArmActions.armToAlgaeBarge(arm));


    // * real competition bindings *

    // home arm
    joystick.rightStick().whileTrue(arm.goToHome())
      .whileTrue(gripper.neutralCommand());
    // joystick.y().whileTrue(gripper.spitOutCommand()).onFalse(gripper.neutralCommand());

    // coral dropoff 
      //manual dunk
    joystick.povLeft().onTrue(ArmActions.dunkCoral(arm, () -> scoringSubsystem.getArmReefTarget(), () -> (joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis())))
      .onTrue(gripper.coralDefaultCommand());
    joystick.leftBumper().onTrue(gripper.spitOutCommand())
      .onFalse(new WaitCommand(0.4).andThen(gripper.neutralCommand())).onFalse(new WaitCommand(0.2).andThen(arm.goToHome()));

      // autodunk
    joystick.povUp().onTrue(ArmActions.dunkDropCoral(arm, gripper, () -> scoringSubsystem.getArmReefTarget()));

    // coral pickup
    joystick.povDown().onTrue(ArmActions.grabFromFunnel(arm, gripper)).onTrue(bucket.disableAutoBucket());
    // bucketHasCoralTrigger.onTrue(ArmActions.grabFromFunnel(arm, gripper));

    // algae pickup
    joystick.povRight().onTrue(ArmActions.removeAlgae(arm, gripper, () -> (((scoringSubsystem.getBranch() / 2) % 2) == 0)));

    // algae dropoff
    // joystick.povUp().whileTrue(ArmActions.armToAlgaeBarge(arm))
    //   .onFalse(ArmActions.shootAlgaeBarge(arm, gripper));

    joystick.y().onTrue(ArmActions.armToProcessor(arm, gripper));

    // wrist fix offset
    joystick.back().onTrue(new InstantCommand(() -> arm.addToWristOffset(Units.degreesToRotations(10))));
    joystick.start().onTrue(new InstantCommand(() -> arm.addToWristOffset(Units.degreesToRotations(-10))));

    // climb
    // joystick.back().onTrue(new ArmCommand(arm, () -> 11)).onTrue(climb.deploy());
    // joystick.start().and(() -> !climb.climbed()).whileTrue(climb.climb()).onFalse(climb.stopCommand());


    /* autodrive TODO: rebind to not conflict with drive stick */
    joystick.b().whileTrue(Autos.getAutoDriveCommandReef(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedBranch(),
    ()->scoringSubsystem.getLevel(),
    ()-> false,
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));
    joystick.rightBumper().whileTrue(Autos.getAutoDriveCommandReef(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedBranch(),
    ()->scoringSubsystem.getLevel(),
    ()-> false,
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));

    joystick.a().whileTrue(Autos.getAutoDriveCommandStation(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedCoralStation(),
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));

    joystick.x().whileTrue(Autos.getAutoDriveCommandReef(drivetrain,
    () -> drivetrain.getState().Pose,
    () -> scoringSubsystem.getRobotPoseForSelectedAlgae(),
    ()->scoringSubsystem.getLevel(),
    ()-> false,
    ()->-joystick.getRightY(),
    ()->-joystick.getRightX(),
    ()->-joystick.getLeftX()));



    // final Command noBlinkPattern = LEDs.runPattern(
    //   () -> LEDPattern.steps(
    //   Map.of(
    //     0,
    //     LEDs.updateStepColor(hasCoral)[0]
    //   )
    // )
    // // .scrollAtRelativeSpeed(Percent.per(Second).of(Constants.LEDConstants.scrollSpeed))
    // );
    // final Command blinkPattern = LEDs.runPattern(
    //   () -> LEDPattern.steps(
    //   Map.of(
    //     0,
    //     LEDs.updateStepColor(hasCoral)[0] 
    //   )
    // ).blink(Seconds.of(Constants.LEDConstants.blinkSeconds))
    // // .scrollAtRelativeSpeed(Percent.per(Second).of(Constants.LEDConstants.scrollSpeed))
    // );

    // if (detectedCoral.get()) {
    //   joystick.back().onTrue(blinkPattern);
    // } else {
    //   joystick.back().onTrue(noBlinkPattern);
    // }
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
  private void configureLEDs() {
    LEDs = new LEDSubsytem();
    if((bucketHasCoral.getAsBoolean() || gripperHasGamePiece.getAsBoolean()) && driveTrainFinishedMoving.getAsBoolean()){
        joystick.rightBumper().onFalse(LEDs.runPattern(
          () -> LEDPattern.steps(
          Map.of(
            0,
            LEDs.getPattern(driveTrainFinishedMoving, bucketHasCoral, gripperHasGamePiece) 
          )
        )//.blink(Seconds.of(Constants.LEDConstants.blinkSeconds))
        ));
      
    } else {
      joystick.rightBumper().onFalse(
        LEDs.runPattern(
      () -> LEDPattern.steps(
      Map.of(
        0,
        LEDs.getPattern(driveTrainFinishedMoving, bucketHasCoral, gripperHasGamePiece) 
      )
      )
    )
      );
    
    }
    bucketHasCoralTrigger.or(driveTrainFinishedMoving).and(() -> DriverStation.isTeleop())
      .onTrue(LEDs.blink()).onFalse(LEDs.breathe());
    

  }
}