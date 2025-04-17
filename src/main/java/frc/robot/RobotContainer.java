// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Vision.USE_QUESTNAV;
import static frc.robot.Constants.Vision.coralStationOffSetsMap;

import java.lang.reflect.Field;
import java.rmi.dgc.Lease;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.Constants.AutoDropoff;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.QuestNav.NerdQuestNav;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.Constants.CoralConstants.coralState;
import frc.robot.Constants.CoralConstants.elevatorLevel;
import frc.robot.commandSequences.Autos;
import frc.robot.commandSequences.SubsystemActions;
import frc.robot.commands.DriveBetweenCages;
import frc.robot.commands.DriveToCoral;
import frc.robot.commands.DriveToCoralAuto;
import frc.robot.commands.DriveToLine;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.PoseEstimatorQuestSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulator;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

  private final ProfiledPIDController driveController = AutoDropoff.driveController;
  private final ProfiledPIDController thetaController = AutoDropoff.thetaController;
    
  private LEDSubsytem LEDs;
  private Climb climb;
  private CoralManipulator coralManipulator;

  // private NerdQuestNav QuestNav = new NerdQuestNav(new Transform3d(0,0, 0, new Rotation3d(Rotation2d.fromDegrees(-90))));

  public static BooleanSupplier autoBucketEnabled = () -> true;

  private BooleanSupplier driveTrainFinishedMoving = () -> false;
    private Trigger driveTrainFinishedMovingTrigger = new Trigger(driveTrainFinishedMoving);
    public boolean aimAssistEnabled = true;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
             .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    // private final Telemetry logger = new Telemetry(MaxSpeed);




    public final CommandSwerveDrivetrain drivetrain;



    public final PoseEstimatorSubsystem poseEstimatorSubsystem;// = new PoseEstimatorSubsystem(drivetrain);

    // public final PoseEstimatorQuestSubsystem poseEstimatorQuestSubsystem;

    // public final ScoringProfileSubsystem scoringSubsystem = new ScoringProfileSubsystem();

    // public final ScoringProfileSubsystem scoringSubsystem;



  /* Path follower */
  private SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveController.setTolerance(Constants.Vision.TRANSLATION_TOLERANCE_X, Constants.Vision.VELOCITY_TOLERANCE_X);
    thetaController.setTolerance(Constants.Vision.ROTATION_TOLERANCE,Constants.Vision.VELOCITY_TOLERANCE_OMEGA);      
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    drivetrain = TunerConstants.createDrivetrain();
    poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);
    // poseEstimatorQuestSubsystem = new PoseEstimatorQuestSubsystem(QuestNav);
    
    // scoringSubsystem = new ScoringProfileSubsystem();
    // climb = new Climb();
    coralManipulator = new CoralManipulator();
    climb = new Climb();

    // configureTriggers();
    configureNamedCommands();


    // SignalLogger.start();
    configureDefaultCommands();

    configureBindings();
    
    // drivetrain.resetPose(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L0).plus(new Transform2d(0.1,0.1,new Rotation2d())));
    configureAutoChooser();
    configureLEDs();


  }
  private Command waitUntilCoralInElevator() {
    return new WaitUntilCommand(() -> !coralManipulator.getIndexerSensor() && !coralManipulator.getIntakeSensor() && coralManipulator.getCoralState().equals(coralState.coralInIndexer));
  }

  private void configureNamedCommands(){
    NamedCommands.registerCommand("resetSubsystems", SubsystemActions.resetDeploy(coralManipulator).andThen(coralManipulator.intakeToHome()).alongWith(climb.returnToHome()).alongWith(coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.visionClear.height)).alongWith(coralManipulator.setCoralStateCommand(coralState.coralInIndexer)));
    NamedCommands.registerCommand("intake", coralManipulator.intakeCommand().alongWith(coralManipulator.setCoralStateCommand(coralState.empty)));
    NamedCommands.registerCommand("waitUntilHasCoral", new WaitUntilCommand(() -> coralManipulator.getCoralState().equals(coralState.coralInIntake)).withTimeout(3.0));
    NamedCommands.registerCommand("waitUntilCoralInRange", new WaitUntilCommand(() -> poseEstimatorSubsystem.coralInRange()).withTimeout(3.0));
    NamedCommands.registerCommand("elevatorToL2", coralManipulator.setElevatorPosition(elevatorLevel.l2.height));
    NamedCommands.registerCommand("elevatorShootL2", SubsystemActions.placeCoralAuto(coralManipulator, elevatorLevel.l2));
    NamedCommands.registerCommand("elevatorToL1", coralManipulator.setElevatorPosition(elevatorLevel.l1inside.height));
    NamedCommands.registerCommand("elevatorSpitL1", coralManipulator.shoot(elevatorLevel.l1inside.shootVoltage));
    NamedCommands.registerCommand("elevatorShootL1", SubsystemActions.placeCoralAuto(coralManipulator, elevatorLevel.l1));
    NamedCommands.registerCommand("elevatorShootL1Corner", SubsystemActions.placeCoralAuto(coralManipulator, elevatorLevel.l1corner));
    NamedCommands.registerCommand("elevatorShootL1Inside", SubsystemActions.placeCoralAuto(coralManipulator, elevatorLevel.l1inside));
    NamedCommands.registerCommand("elevatorToHome", coralManipulator.setElevatorPosition(elevatorLevel.visionClear.height));
    NamedCommands.registerCommand("elevatorToHomeAndIntake", coralManipulator.setElevatorPosition(elevatorLevel.visionClear.height).alongWith(coralManipulator.intakeCommand()));
    NamedCommands.registerCommand("elevatorShootL2", SubsystemActions.placeCoralAuto(coralManipulator, elevatorLevel.l2));
    NamedCommands.registerCommand("driveToCoral", new DriveToCoralAuto(drivetrain, () -> (poseEstimatorSubsystem.coralArrayUpdateReturn().size() > 0) ? poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose() : getQuestPose()).withTimeout(2));
    NamedCommands.registerCommand("waitUntilCoralInElevator", waitUntilCoralInElevator());
    NamedCommands.registerCommand("waitUntilAndElevatorL2", waitUntilCoralInElevator().andThen(coralManipulator.setElevatorPosition(elevatorLevel.l2.height).alongWith(coralManipulator.setIntakeVoltage(-5).alongWith(coralManipulator.intakeToDeploy()))).withTimeout(1.0));
    NamedCommands.registerCommand("waitUntilAndElevatorL1", waitUntilCoralInElevator().andThen(coralManipulator.setElevatorPosition(elevatorLevel.l1.height).alongWith(coralManipulator.setIntakeVoltage(-5).alongWith(coralManipulator.intakeToDeploy()))).withTimeout(1.0));
    NamedCommands.registerCommand("waitUntilAndShootL2", waitUntilCoralInElevator().andThen(SubsystemActions.placeCoralAuto(coralManipulator, elevatorLevel.l2).alongWith(coralManipulator.setIntakeVoltage(-5).alongWith(coralManipulator.intakeToDeploy()))).withTimeout(1.0));
    NamedCommands.registerCommand("waitUntilAndShootL1", waitUntilCoralInElevator().andThen(SubsystemActions.placeCoralAuto(coralManipulator, elevatorLevel.l1inside).alongWith(coralManipulator.setIntakeVoltage(-5).alongWith(coralManipulator.intakeToDeploy()))).withTimeout(1.0));
    NamedCommands.registerCommand("waitUntilAndShootL1Inside", waitUntilCoralInElevator().andThen(SubsystemActions.placeCoralAuto(coralManipulator, elevatorLevel.l1inside).alongWith(coralManipulator.setIntakeVoltage(-5).alongWith(coralManipulator.intakeToDeploy()))).withTimeout(1.0));
    NamedCommands.registerCommand("waitUntilAndElevatorL1Inside", waitUntilCoralInElevator().andThen(coralManipulator.setElevatorPosition(elevatorLevel.l1inside.height).alongWith(coralManipulator.setIntakeVoltage(-5).alongWith(coralManipulator.intakeToDeploy()))).withTimeout(1.0));

  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(xLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY()) * MaxSpeed))
          .withVelocityY(yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed))
          .withRotationalRate(zLimiter.calculate(-joystick.getLeftX() * MaxAngularRate))
        )
        );

  }
  // private void configureTriggers() {
  //   // driveTrainFinishedMoving = () -> poseEstimatorSubsystem.getCurrentPose().getTranslation()
  //   // .getDistance(scoringSubsystem.getSelectedBranchPose().getTranslation()) < 1;
  //   //  || poseEstimatorSubsystem.getCurrentPose().getTranslation().getDistance((scoringSubsystem.getSelectedCoralStationPose().getTranslation()))<1;
  //   // bucketHasCoralTrigger = new Trigger(bucketHasCoral);
    
  // }


  private Pose2d getQuestPose() {
    return /*USE_QUESTNAV ? poseEstimatorQuestSubsystem.getCurrentPose() :*/ drivetrain.getState().Pose;
  } 
  private void disableAimAssist(boolean enable) {
    aimAssistEnabled = enable;
  }
  private void configureBindings() {
    // Find Quest Offsets
    //joystick.leftTrigger().onTrue(QuestNav.determineOffsetToRobotCenter(drivetrain, 0.35)); //0.314

    // joystick.b().onTrue(new InstantCommand(() -> disableAimAssist(false)));
    // joystick.x().onTrue(new InstantCommand(() -> disableAimAssist(true)));

    // climb
    copilot.y().onTrue(climb.extend().alongWith(coralManipulator.intakeToDeploy()));
    copilot.x().onTrue(climb.returnToHome());
    copilot.a().onTrue(climb.contract().alongWith(coralManipulator.stopDeploy()));
    new Trigger(() -> climb.getPosition() > ClimbConstants.climbedPosition).onTrue(climb.stopCommand());
    copilot.b().onTrue(climb.stopCommand());

    //reset buttons
    joystick.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    copilot.back().onTrue(SubsystemActions.resetDeploy(coralManipulator)).onTrue(SubsystemActions.resetElevator(coralManipulator));//..onFalse(coralManipulator.stopElevator()).onFalse(coralManipulator.intakeToHome());

    // panic button
    joystick.y().onTrue(SubsystemActions.panicButton(coralManipulator))
      .onFalse(coralManipulator.intakeToHome().alongWith(coralManipulator.stopIntake()).alongWith(coralManipulator.stopIndexer()).alongWith(coralManipulator.stopShooter()).alongWith(coralManipulator.elevatorToHome()));

    joystick.back().whileTrue(new DriveBetweenCages(
        drivetrain, 
        () -> new Pose2d(new Translation2d(0, FieldConstants.getClosestBargeGap(() -> getQuestPose())), Rotation2d.kCCW_90deg), 
        () -> new Translation2d(OperatorConstants.joystickMap.get(-joystick.getRightY()), OperatorConstants.joystickMap.get(-joystick.getRightX()))));

    // full auto dropoffs for L2
    joystick.povUp().and(() -> FieldConstants.getCloseEnoughForAutoDrive(() -> drivetrain.getState().Pose))
      .whileTrue(coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.visionClear.height)
        .andThen(new DriveToPose(drivetrain, () -> FieldConstants.getClosestPole(() -> drivetrain.getState().Pose)))
        .andThen(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l2)));
        // .and(() -> coralManipulator.getCoralState().equals(coralState.coralInElevator))
          // .onTrue(coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.l2.height))
          // .onFalse(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l2));
    // joystick.povUp()
    // .whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l2)).onFalse(coralManipulator.elevatorToHome());

    // semi auto dropoffs for L1
    joystick.leftBumper().and(() -> FieldConstants.getCloseEnoughForAutoDrive(() -> drivetrain.getState().Pose))
      // .whileTrue(new DriveToLine(
      //   drivetrain, 
      //   () -> FieldConstants.getClosestL1(() -> drivetrain.getState().Pose), 
      //   () -> new Translation2d(OperatorConstants.joystickMap.get(-joystick.getRightY()), OperatorConstants.joystickMap.get(-joystick.getRightX()))))
        .whileTrue(new DriveToPose(drivetrain, () -> FieldConstants.getClosestL1(() -> drivetrain.getState().Pose).transformBy(new Transform2d(new Translation2d(copilot.getLeftX(), Constants.Vision.reefLevelOffsetsMap.get(ReefLevel.L1).getRotation().plus(Rotation2d.kCCW_90deg)),new Rotation2d()))))
      .and(() -> coralManipulator.getCoralState().equals(coralState.coralInIndexer))
        .onTrue(coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.visionClear.height));

    joystick.povRight()
      .whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1corner));
    joystick.povLeft()
      .whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1upper));
    joystick.povDown()
      .whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1));

    joystick.povRight().or(joystick.povLeft()).or(joystick.povDown()).or(joystick.povUp()).and(() -> !coralManipulator.getCoralState().equals(coralState.coralInIndexer)).onFalse(coralManipulator.elevatorToHome());

    //intake commands
    joystick.rightTrigger()
      .onTrue(coralManipulator.setCoralStateCommand(coralState.empty))
      .onTrue(coralManipulator.intakeCommand());//.onlyIf(() -> coralManipulator.getCoralState().equals(coralState.empty)))
      // .onFalse(coralManipulator.intakeToHome());//.onlyIf(() -> !coralManipulator.getCoralState().equals(coralState.coralInIntake) ));
    joystick.x().onTrue(coralManipulator.intakeToHome());

    joystick.rightBumper().onTrue(SubsystemActions.intakeAlgae(coralManipulator)).onFalse(coralManipulator.intakeToAlgaeHome().alongWith(coralManipulator.setIntakeVoltage(CoralConstants.neutralAlgaeVoltage)));
    new Trigger(() -> coralManipulator.getCoralState().equals(coralState.algaeInIntake) && coralManipulator.getIntakeSensor()).onTrue(new WaitCommand(0.5).andThen(coralManipulator.intakeToAlgaeHome().alongWith(coralManipulator.setIntakeVoltage(CoralConstants.neutralAlgaeVoltage))));
    joystick.start().onTrue(SubsystemActions.prepareDropOffAlgae(coralManipulator)).onFalse(SubsystemActions.dropOffAlgae(coralManipulator));


      Trigger coralInRange = new Trigger(() -> poseEstimatorSubsystem.coralInRange());
      // Trigger coralAutoTarget = new Trigger(() -> Constants.Vision.kCoralAutoTarget);
      Trigger coralInList = new Trigger(() -> poseEstimatorSubsystem.coralInList());
      


          // AIMASSIST
      // coralInList
      // .and(coralInRange)
      // .and(() -> joystick.getLeftTriggerAxis() < 0.2)
      // .and(() -> coralManipulator.getCoralState()
      // .equals(coralState.empty)).and(() -> joystick.rightTrigger().getAsBoolean())
      // .and(() -> aimAssistEnabled)
        // .whileTrue(
        //   drivetrain.applyRequest(() ->
        //     drive
        //     .withVelocityX(xLimiter.calculate(Math.hypot(OperatorConstants.joystickMap.get(-joystick.getRightY() * MaxSpeed), (OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed)) * poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation().getCos()))
        //     .withVelocityY(yLimiter.calculate(Math.hypot((OperatorConstants.joystickMap.get(-joystick.getRightY()) * MaxSpeed), (OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed)) * poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation().getSin()))
            // .withVelocityX(
            //   Math.hypot(xLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY()) * MaxSpeed), yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed))
            //    * Math.cos(poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getTranslation().getAngle().getRadians() - (Math.atan2(yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX())), yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY())))))
            //    * poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getTranslation().getAngle().getCos())
            //    .withVelocityY(
            //     Math.hypot(xLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY()) * MaxSpeed), yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed))
            //      * Math.cos(poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getTranslation().getAngle().getRadians() - (Math.atan2(yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX())), yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY())))))
            //      * poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getTranslation().getAngle().getSin())
            //   .withRotationalRate( MathUtil.clamp(10* (drivetrain.getState().Pose.getRotation().getRadians()-poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation().getRadians() > Math.PI ? 
            //     drivetrain.getState().Pose.getRotation().getRadians()-poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation().getRadians() - (2.0 * Math.PI) : 
            //     (drivetrain.getState().Pose.getRotation().getRadians()-poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation().getRadians() < -Math.PI ? 
            //     drivetrain.getState().Pose.getRotation().getRadians()-poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation().getRadians() + (2.0 * Math.PI) : 
            //     drivetrain.getState().Pose.getRotation().getRadians()-poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation().getRadians())
            //   ), -5, 5))
            // ));
        
            // DRIVETOCORAL
          joystick.leftTrigger().and(coralInRange).and(coralInList)
        .whileTrue(new DriveToCoral(drivetrain,
          () -> new Pose2d(
            poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getX() + new Translation2d(-0.2, poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation()).getX(),
            poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getY() + new Translation2d(-0.2, poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation()).getY(),
            poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose().getRotation()), 
          () -> getQuestPose()))
          .onTrue(coralManipulator.intakeCommand());

      
    new Trigger(() -> coralManipulator.getCoralState().equals(coralState.coralInIntake)).and(() -> DriverStation.isTeleop()).onTrue(SubsystemActions.transferCoral(coralManipulator));

    new Trigger(() -> coralManipulator.getCoralState().equals(coralState.coralInIntake)).and(() -> DriverStation.isAutonomous()).onTrue(new WaitCommand(0.0).andThen(SubsystemActions.transferCoralForAuto(coralManipulator)));
    // new Trigger(() -> !coralManipulator.getIndexerSensor()).onTrue(coralManipulator.setCoralStateCommand(coralState.coralInIndexer));
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

    new Trigger(coralManipulator::getIntakeSensor).whileTrue(LEDs.runPattern(() -> LEDPattern.solid(new Color(0d, 0d, 1d))));
    new Trigger(coralManipulator::getIndexerSensor).whileTrue(LEDs.runPattern(() -> LEDPattern.solid(new Color(0d, 1d, 0d))));
    LEDs.setDefaultCommand(LEDs.runPattern(() -> LEDPattern.solid(new Color(1, 0, 0))));
    // if (!climb.climbed()) {
      // LEDs.setDefaultCommand(LEDs.runPattern(() -> LEDPattern.solid(LEDs.getColor(() -> true, () -> joystick.rightBumper().getAsBoolean(), () -> 1.0))));
    // } else {
    //   LEDs.setDefaultCommand(
    //     LEDs.runPattern(() -> LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(25)))
    //   );
    // }

  }
  // private Command driveToLine(Supplier<Pose2d> targetCenter, Supplier<Translation2d> joystickVector, Supplier<Rotation2d> targetVector) {
  //   // double target = new Translation2d(targetCenter.get().getTranslation().getX() * targetVector.get().getCos(), targetCenter.get().getTranslation().getY() * targetVector.get().getSin()).getNorm();
  //   return drivetrain.applyRequest(() ->
  //   drive
  //     .withVelocityX(xLimiter.calculate((OperatorConstants.joystickMap.get(joystickVector.get().getX() * targetVector.get().getCos()) * MaxSpeed) + driveController.calculate(
  //       new Translation2d(
  //         drivetrain.getState().Pose.getTranslation().getX() * targetVector.get().plus(Rotation2d.kCCW_90deg).getCos(), 
  //         drivetrain.getState().Pose.getTranslation().getY() * targetVector.get().plus(Rotation2d.kCCW_90deg).getSin()).getNorm(), 
  //       new Translation2d(
  //         targetCenter.get().getTranslation().getX() * targetVector.get().plus(Rotation2d.kCCW_90deg).getCos(), 
  //         targetCenter.get().getTranslation().getY() * targetVector.get().plus(Rotation2d.kCCW_90deg).getSin()).getNorm()
  //     ) * targetVector.get().getSin()))
  //     .withVelocityY(yLimiter.calculate((OperatorConstants.joystickMap.get(joystickVector.get().getY() * targetVector.get().getSin()) * MaxSpeed) 
  //       + driveController.calculate(
  //         new Translation2d(
  //           drivetrain.getState().Pose.getTranslation().getX() * targetVector.get().plus(Rotation2d.kCCW_90deg).getCos(), 
  //           drivetrain.getState().Pose.getTranslation().getY() * targetVector.get().plus(Rotation2d.kCCW_90deg).getSin()).getNorm(), 
  //         new Translation2d(
  //           targetCenter.get().getTranslation().getX() * targetVector.get().plus(Rotation2d.kCCW_90deg).getCos(), 
  //           targetCenter.get().getTranslation().getY() * targetVector.get().plus(Rotation2d.kCCW_90deg).getSin()).getNorm()
  //       ) * targetVector.get().getCos()))
  //     .withRotationalRate(zLimiter.calculate(thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), targetVector.get().getRadians()))));
  // }
}