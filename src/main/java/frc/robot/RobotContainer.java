// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Vision.coralStationOffSetsMap;

import java.lang.reflect.Field;
import java.rmi.dgc.Lease;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.Constants.AutoDropoff;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.Constants.CoralConstants.coralState;
import frc.robot.Constants.CoralConstants.elevatorLevel;
import frc.robot.commandSequences.Autos;
import frc.robot.commandSequences.SubsystemActions;
import frc.robot.commands.DriveToCoral;
import frc.robot.commands.DriveToCoralAuto;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.subsystems.LEDSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulator;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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

  public static BooleanSupplier autoBucketEnabled = () -> true;

  private BooleanSupplier driveTrainFinishedMoving = () -> false;
    private Trigger driveTrainFinishedMovingTrigger = new Trigger(driveTrainFinishedMoving);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
             .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // private final Telemetry logger = new Telemetry(MaxSpeed);




    public final CommandSwerveDrivetrain drivetrain;



    public final PoseEstimatorSubsystem poseEstimatorSubsystem;// = new PoseEstimatorSubsystem(drivetrain);
    

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
    // scoringSubsystem = new ScoringProfileSubsystem();
    // climb = new Climb();
    coralManipulator = new CoralManipulator();

    // configureTriggers();
    configureNamedCommands();


    // SignalLogger.start();
    configureDefaultCommands();

    configureBindings();
    
    // drivetrain.resetPose(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L0).plus(new Transform2d(0.1,0.1,new Rotation2d())));
    // configureAutoChooser();
    configureLEDs();


  }
  
  private void configureNamedCommands(){
    NamedCommands.registerCommand("intake", coralManipulator.intakeCommand());
    NamedCommands.registerCommand("waitUntilHasCoral", new WaitUntilCommand(() -> !coralManipulator.getCoralState().equals(coralState.empty)));
    NamedCommands.registerCommand("elevatorToL2", coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.l2.height));
    NamedCommands.registerCommand("elevatorShootL2", SubsystemActions.placeCoral(coralManipulator, elevatorLevel.l2));
    NamedCommands.registerCommand("elevatorToL1", coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.l1.height));
    NamedCommands.registerCommand("elevatorShootL1", SubsystemActions.placeCoral(coralManipulator, elevatorLevel.l1));
    NamedCommands.registerCommand("elevatorToHome", coralManipulator.elevatorToHome());
    NamedCommands.registerCommand("elevatorShootL2", SubsystemActions.placeCoral(coralManipulator, elevatorLevel.l2));
    NamedCommands.registerCommand("driveToCoral", new DriveToCoralAuto(drivetrain, () -> (poseEstimatorSubsystem.coralArrayUpdateReturn().size() > 0) ? poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose() : poseEstimatorSubsystem.getCurrentPose()));
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(xLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightY()) * MaxSpeed))
          .withVelocityY(yLimiter.calculate(OperatorConstants.joystickMap.get(-joystick.getRightX()) * MaxSpeed))
          .withRotationalRate(zLimiter.calculate(-joystick.getLeftX() * MaxAngularRate))
        )
        );
    
    




    // gripper.setDefaultCommand(new GripperCommand(gripper));



  }
  private void configureTriggers() {
    // driveTrainFinishedMoving = () -> poseEstimatorSubsystem.getCurrentPose().getTranslation()
    // .getDistance(scoringSubsystem.getSelectedBranchPose().getTranslation()) < 1;
    //  || poseEstimatorSubsystem.getCurrentPose().getTranslation().getDistance((scoringSubsystem.getSelectedCoralStationPose().getTranslation()))<1;
    // bucketHasCoralTrigger = new Trigger(bucketHasCoral);
    
  }


  private void configureBindings() {
    joystick.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    joystick.back().whileTrue(SubsystemActions.resetDeploy(coralManipulator).alongWith(SubsystemActions.resetElevator(coralManipulator)));

    joystick.rightBumper().onTrue(SubsystemActions.panicButton(coralManipulator));

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
    joystick.leftBumper()
      .whileTrue(driveToLine(() -> AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[FieldConstants.getClosestFace(() -> drivetrain.getState().Pose)]).plus(Constants.Vision.reefLevelOffsetsMap.get(ReefLevel.L1)), () -> new Translation2d(-joystick.getRightY(), -joystick.getRightX()), () -> FieldConstants.Reef.centerFaces[FieldConstants.getClosestFace(() -> drivetrain.getState().Pose)].getTranslation().minus(FieldConstants.Reef.center).getAngle()))
      .and(() -> coralManipulator.getCoralState().equals(coralState.coralInElevator))
        .whileTrue(coralManipulator.setElevatorPosition(CoralConstants.elevatorLevel.l1.height));

    joystick.povRight()
      .whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1inside));
    joystick.povLeft()
      .whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1upper));
    joystick.povDown()
      .whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1));

    joystick.povRight().or(joystick.povLeft()).or(joystick.povDown()).or(joystick.povUp()).and(() -> !coralManipulator.getCoralState().equals(coralState.coralInElevator)).onFalse(coralManipulator.elevatorToHome());

    //intake commands
    joystick.leftTrigger()
      .onTrue(coralManipulator.setCoralStateCommand(coralState.empty))
      .onTrue(coralManipulator.intakeCommand())//.onlyIf(() -> coralManipulator.getCoralState().equals(coralState.empty)))
      .onFalse(coralManipulator.intakeToHome().onlyIf(() -> coralManipulator.getCoralState().equals(coralState.empty)));

      Trigger coralInRange = new Trigger(() -> poseEstimatorSubsystem.coralInRange());
      Trigger coralAutoTarget = new Trigger(() -> Constants.Vision.kCoralAutoTarget);
      Trigger coralInList = new Trigger(() -> poseEstimatorSubsystem.coralInList());
      
      joystick.leftTrigger().and(coralInRange).and(coralAutoTarget).and(coralInList).whileTrue(new DriveToCoral(drivetrain, () -> poseEstimatorSubsystem.coralArrayUpdateReturn().get(0).getPose()));
      
    new Trigger(() -> coralManipulator.getCoralState().equals(coralState.coralInIntake)).onTrue(SubsystemActions.transferCoral(coralManipulator));

    // new Trigger(() -> coralManipulator.getCoralState().equals(coralState.coralInIntake))
    //   .whileTrue(SubsystemActions.transferCoralToIndexer(coralManipulator))
    //   .onFalse(coralManipulator.stopIndexer().andThen(coralManipulator.stopIntake()).andThen(new WaitCommand(0.2).andThen(coralManipulator.retractIntake())));

    // new Trigger(() -> coralManipulator.getCoralState().equals(coralState.coralInIndexer))
    //   .whileTrue(SubsystemActions.transferCoralToElevator(coralManipulator))
    //   .onFalse(new SequentialCommandGroup(
    //     coralManipulator.setCoralStateCommand(coralState.coralInElevator), 
    //     coralManipulator.stopIndexer(),
    //     coralManipulator.stopShooter(),
    //     coralManipulator.elevatorToHome()));


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
    joystick.b().onTrue(LEDs.runPattern(() -> LEDPattern.solid(new Color(1, 1, 1))));
    // if (!climb.climbed()) {
      // LEDs.setDefaultCommand(LEDs.runPattern(() -> LEDPattern.solid(LEDs.getColor(() -> true, () -> joystick.rightBumper().getAsBoolean(), () -> 1.0))));
    // } else {
    //   LEDs.setDefaultCommand(
    //     LEDs.runPattern(() -> LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(25)))
    //   );
    // }

  }
  private Command driveToLine(Supplier<Pose2d> targetCenter, Supplier<Translation2d> joystickVector, Supplier<Rotation2d> targetVector) {
    // double target = new Translation2d(targetCenter.get().getTranslation().getX() * targetVector.get().getCos(), targetCenter.get().getTranslation().getY() * targetVector.get().getSin()).getNorm();
    return drivetrain.applyRequest(() ->
    drive
      .withVelocityX(xLimiter.calculate((OperatorConstants.joystickMap.get(joystickVector.get().getX() * targetVector.get().getCos()) * MaxSpeed) + driveController.calculate(
        new Translation2d(
          drivetrain.getState().Pose.getTranslation().getX() * targetVector.get().plus(Rotation2d.kCCW_90deg).getCos(), 
          drivetrain.getState().Pose.getTranslation().getY() * targetVector.get().plus(Rotation2d.kCCW_90deg).getSin()).getNorm(), 
        new Translation2d(
          targetCenter.get().getTranslation().getX() * targetVector.get().plus(Rotation2d.kCCW_90deg).getCos(), 
          targetCenter.get().getTranslation().getY() * targetVector.get().plus(Rotation2d.kCCW_90deg).getSin()).getNorm()
      ) * targetVector.get().getSin()))
      .withVelocityY(yLimiter.calculate((OperatorConstants.joystickMap.get(joystickVector.get().getY() * targetVector.get().getSin()) * MaxSpeed) 
        + driveController.calculate(
          new Translation2d(
            drivetrain.getState().Pose.getTranslation().getX() * targetVector.get().plus(Rotation2d.kCCW_90deg).getCos(), 
            drivetrain.getState().Pose.getTranslation().getY() * targetVector.get().plus(Rotation2d.kCCW_90deg).getSin()).getNorm(), 
          new Translation2d(
            targetCenter.get().getTranslation().getX() * targetVector.get().plus(Rotation2d.kCCW_90deg).getCos(), 
            targetCenter.get().getTranslation().getY() * targetVector.get().plus(Rotation2d.kCCW_90deg).getSin()).getNorm()
        ) * targetVector.get().getCos()))
      .withRotationalRate(zLimiter.calculate(thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), targetVector.get().getRadians()))));
  }
}