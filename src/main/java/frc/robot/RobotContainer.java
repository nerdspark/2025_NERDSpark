// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.*;

import java.rmi.dgc.Lease;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.CoralConstants.coralState;
import frc.robot.commandSequences.Autos;
import frc.robot.commandSequences.SubsystemActions;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ScoringProfileSubsystem;
import frc.robot.subsystems.Vision;
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



    // public final PoseEstimatorSubsystem poseEstimatorSubsystem;// = new PoseEstimatorSubsystem(drivetrain);
    

    // public final ScoringProfileSubsystem scoringSubsystem = new ScoringProfileSubsystem();

    // public final ScoringProfileSubsystem scoringSubsystem;



  /* Path follower */
  private SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrain = TunerConstants.createDrivetrain();
    // poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);
    // scoringSubsystem = new ScoringProfileSubsystem();
    // climb = new Climb();
    coralManipulator = new CoralManipulator();

    // configureTriggers();
    // configureNamedCommands();


    // SignalLogger.start();
    configureDefaultCommands();

    configureBindings();
    
    // drivetrain.resetPose(FieldConstants.Reef.branchPositions2d.get(0).get(ReefLevel.L0).plus(new Transform2d(0.1,0.1,new Rotation2d())));
    // configureAutoChooser();
    // configureLEDs();


  }
  
  private void configureNamedCommands(){
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
    joystick.povUp().whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l2)).onFalse(coralManipulator.elevatorHome());//stopElevator().alongWith(elevIndexer.stopShooter()));
    joystick.povRight().whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1inside)).onFalse(coralManipulator.elevatorHome());//stopElevator().alongWith(elevIndexer.stopShooter()));
    joystick.povLeft().whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1upper)).onFalse(coralManipulator.elevatorHome());//stopElevator().alongWith(elevIndexer.stopShooter()));
    joystick.povDown().whileTrue(SubsystemActions.placeCoral(coralManipulator, CoralConstants.elevatorLevel.l1)).onFalse(coralManipulator.elevatorHome());//stopElevator().alongWith(elevIndexer.stopShooter()));
    
    new Trigger(() -> coralManipulator.getIntakeSensor() && coralManipulator.getCoralState().equals(coralState.coralInIntake)).onTrue(SubsystemActions.transferCoralToIndexer(coralManipulator));
    new Trigger(() -> coralManipulator.getIndexerSensor() && coralManipulator.getCoralState().equals(coralState.coralInIndexer)).onTrue(SubsystemActions.transferCoralToElevator(coralManipulator));

    joystick.rightBumper().whileTrue(new DriveToPose(drivetrain, () -> FieldConstants.getReefPoseOffset(
      () -> joystick.getRightX(), FieldConstants.getClosestFace(() -> drivetrain.getState().Pose))));    

    // joystick.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


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

    if (!climb.climbed()) {
      LEDs.setDefaultCommand(LEDs.runPattern(() -> LEDPattern.solid(LEDs.getColor(() -> true, () -> joystick.rightBumper().getAsBoolean(), () -> 0.0))));
    } else {
      LEDs.setDefaultCommand(
        LEDs.runPattern(() -> LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(25)))
      );
    }

  }
}