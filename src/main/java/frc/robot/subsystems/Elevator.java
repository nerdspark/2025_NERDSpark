// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

// TODO: Create a command for Elevator Subsystem

public class Elevator extends SubsystemBase {
  private TalonFX leftMotor = new TalonFX(0);
  private TalonFX rightMotor = new TalonFX(1);

  /** Creates a new Elevator. */
  public Elevator() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = ElevatorConstants.kElevatorkS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = ElevatorConstants.kElevatorkV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ElevatorConstants.kElevatorkA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = ElevatorConstants.kElevatorKp; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = ElevatorConstants.kElevatorKi; // no output for integrated error
    slot0Configs.kD = ElevatorConstants.kElevatorKd; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.kCruiseVelocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.kAccel; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = ElevatorConstants.kJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    leftMotor.getConfigurator().apply(talonFXConfigs);
    rightMotor.setControl(new Follower(0, false)); // TODO: Change MasterID

  }


  public void setPosition(double target) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(ElevatorConstants.kMinElevatorHeightRotations).withSlot(0); // TODO: Change Slot
    double targetPosition = MathUtil.clamp(target, ElevatorConstants.kMinElevatorHeightRotations, ElevatorConstants.kMaxElevatorHeightRotations);
    leftMotor.setControl(m_request.withPosition(targetPosition));
  }

  public double getPositionMeters() {
    return leftMotor.getPosition().getValueAsDouble() * (2 * Math.PI * Constants.ElevatorConstants.kElevatorDrumRadius) / Constants.ElevatorConstants.kElevatorGearing;
  }

  public double getVelocityMetersPerSecond() {
    return (leftMotor.getVelocity().getValueAsDouble() / 60) * (2 * Math.PI * Constants.ElevatorConstants.kElevatorDrumRadius)
            / Constants.ElevatorConstants.kElevatorGearing;
  }

  public void stop() {
    leftMotor.stopMotor();
  }

  public void setPower(double power) {
    leftMotor.setControl(new DutyCycleOut(power));
  }   

  public boolean isOnTarget(double currentPosition, double targetPosition) {
    return Math.abs(targetPosition - currentPosition) <= ElevatorConstants.kElevatorDefaultTolerance;
  }

  public double getCurrentPower() {
    return leftMotor.get();
  }

  public double getMotorCurrent() {
    return leftMotor.getStatorCurrent().getValueAsDouble();
  }   




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position (Rotations)", leftMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Velocity (Rotations per Second)", leftMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Position (Meters)", getPositionMeters());
    SmartDashboard.putNumber("Elevator Velocity (Meters per Second)", getVelocityMetersPerSecond());
  }
}





// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Meter;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Volts;

// import java.util.function.BooleanSupplier;
// import java.util.function.Supplier;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.servohub.ServoHub.ResetMode;
// import com.revrobotics.sim.SparkMaxSim;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.units.measure.MutDistance;
// import edu.wpi.first.units.measure.MutLinearVelocity;
// import edu.wpi.first.units.measure.MutVoltage;
// import edu.wpi.first.wpilibj.Alert;
// import edu.wpi.first.wpilibj.Alert.AlertType;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.DIOSim;
// import edu.wpi.first.wpilibj.simulation.ElevatorSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.Constants;
// import frc.robot.Constants.ElevatorConstants;


// public class Elevator extends SubsystemBase {
//     private final TalonFX m_motor;
//     private final TalonFX m_motorRight;   
//     private final RelativeEncoder m_encoder;
//     private final ProfiledPIDController m_controller;        
//     private final ElevatorFeedforward m_feedForward;
//     private final BooleanSupplier atMin;
//     private final BooleanSupplier atMax;
//     private final SysIdRoutine m_SysIdRoutine;
//     private final MutVoltage m_appliedVoltage;
//     private final MutDistance m_distance;
//     private final MutLinearVelocity m_velocity;
 
//     private final DigitalInput m_limitSwitchLow;

//     // sim stuff
//     private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
//     private ElevatorSim m_elevatorSim = null;
//     private DIOSim m_limitSwitchLowSim = null;


    
//     // You'll need to adjust this based on your elevator's gearing
//     // private final double COUNTS_PER_INCH = 42.0; // Example value - measure this!
    
//     public Elevator() { // Replace SparkMax CANID
//         // m_motor = new SparkMax(0, MotorType.kBrushless);
//         // m_motorRight = new SparkMax(1, MotorType.kBrushless);
//         m_motor = new TalonFX(0, "*");
//         m_motorRight = new TalonFX(1, "*");

//         // m_encoder = m_motor.getEncoder(); TODO: Find an encoder for the TalonFX motor
        
//         // pid values need tuning, change maxVelocity and maxAcceleration of trapezoid profile
//         m_controller = new ProfiledPIDController(
//             Constants.ElevatorConstants.kElevatorKp,
//             Constants.ElevatorConstants.kElevatorKi,
//             Constants.ElevatorConstants.kElevatorKd,
//             new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration)
//         );
//         m_feedForward = new ElevatorFeedforward(
//             Constants.ElevatorConstants.kElevatorkS,
//             Constants.ElevatorConstants.kElevatorkG,
//             Constants.ElevatorConstants.kElevatorkV,
//             Constants.ElevatorConstants.kElevatorkA
//         ); 

//         atMin = () -> MathUtil.isNear(getPositionMeters(), ElevatorConstants.kMinElevatorHeightMeters, Inches.of(1).in(Meters));
//         atMax = () -> MathUtil.isNear(getPositionMeters(), ElevatorConstants.kMaxElevatorHeightMeters, Inches.of(1).in(Meters));

//         m_appliedVoltage = Volts.mutable(0);
//         m_distance = Meters.mutable(0);
//         m_velocity = MetersPerSecond.mutable(0);

//         // m_SysIdRoutine = new SysIdRoutine(
//         //   // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
//         //   new SysIdRoutine.Config(Volts.per(Second).of(2),
//         //                           Volts.of(2),
//         //                           Seconds.of(30)),
//         // //   new SysIdRoutine.Mechanism(
//         // //       // Tell SysId how to plumb the driving voltage to the motor(s).
//         // //       m_motor::setVoltage,
//         // //       // Tell SysId how to record a frame of data for each motor on the mechanism being
//         // //       // characterized.
//         // //       log -> {
//         // //         // Record a frame for the shooter motor.
//         // //         log.motor("elevator")
//         // //            .voltage(
//         // //                m_appliedVoltage.mut_replace(
//         // //                    m_motor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
//         // //            .linearPosition(m_distance.mut_replace(getPositionMeters(),
//         // //                                                   Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
//         // //            .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSecond(),
//         // //                                                   MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
//         // //       },
//         // //       this
//         // //     ) Sys ID routine????
//         // );

//         // TODO: apply config to left and right motors

//         m_limitSwitchLow = new DigitalInput(9);


        
//         if (RobotBase.isSimulation()) {
//             m_elevatorSim = new ElevatorSim(
//                 m_elevatorGearbox,
//                 ElevatorConstants.kElevatorGearing,
//                 ElevatorConstants.kCarriageMass,
//                 ElevatorConstants.kElevatorDrumRadius,
//                 ElevatorConstants.kMinElevatorHeight.in(Meters),
//                 ElevatorConstants.kMaxElevatorHeight.in(Meters),
//                 true,
//                 ElevatorConstants.kStartingHeightSim.in(Meters),
//                 0.01,
//                 0.0
//             );
//             m_limitSwitchLowSim = new DIOSim(m_limitSwitchLow);
//             SmartDashboard.putData("Elevator Low Limit Switch", m_limitSwitchLow);  
//         }
        

//     }

//     // Returns elevator height in meters
//     public double getPositionMeters() {
//         return m_encoder.getPosition() * (2 * Math.PI * Constants.ElevatorConstants.kElevatorDrumRadius) / Constants.ElevatorConstants.kElevatorGearing;
//     }

//     public double getVelocityMetersPerSecond() {
//         return (m_encoder.getVelocity() / 60) * (2 * Math.PI * Constants.ElevatorConstants.kElevatorDrumRadius)
//                 / Constants.ElevatorConstants.kElevatorGearing;
//     }

//     public void reachGoal(double goal) {
//         // Change low and high values of voltage output
//         double voltsOutput = MathUtil.clamp(
//                 m_feedForward.calculateWithVelocities(getVelocityMetersPerSecond(), m_controller.getSetpoint().velocity) + m_controller.calculate(getPositionMeters(), goal),
//                 -7,     /* low */
//             7     /* high */  );
//         m_motor.setVoltage(voltsOutput);
//     }

//      public Command setGoal(double goal){
//         return run(() -> reachGoal(goal));
//     }

//     public Command setElevatorHeight(double height){
//         return setGoal(height).until(()->aroundHeight(height));
//     }


//     // we could prob condense these two functions into one
//     public boolean aroundHeight(double height){
//         return findIfAroundHeight(height, Constants.ElevatorConstants.kElevatorDefaultTolerance);
//     }

//     public boolean findIfAroundHeight(double height, double tolerance){
//         return MathUtil.isNear(height,getPositionMeters(),tolerance);
//     }

//     public void stop() {
//         m_motor.set(0.0);
//     }

//     // sets the speed of the speed controller
//     public Command setPower(double d) {
//         return run(() -> m_motor.set(d));
//     }

//     private double holdPoint = 0.0;

//     public Command hold() {
//         return startRun(() -> {
//             holdPoint = MathUtil.clamp(getPositionMeters(), 0.01, 6);
//             m_controller.reset(holdPoint);
//         }, () -> reachGoal(MathUtil.clamp(holdPoint,ElevatorConstants.kMinElevatorHeightMeters,ElevatorConstants.kMaxElevatorHeightMeters)));
//     }

//     public Command goToL1() {
//         // setElevatorHeight(ElevatorConstants.kHeightl1);
//         return setPower(-0.1).until(atMin);
//     }

//     public Command goToL2() {
//         return setElevatorHeight(ElevatorConstants.kHeightL2);
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Elevator Position (Rotations)", m_encoder.getPosition());
//         SmartDashboard.putNumber("Elevator Velocity (Rotations per Second)", m_encoder.getVelocity());
//         SmartDashboard.putNumber("Elevator Position (Meters)", getPositionMeters());
//         SmartDashboard.putNumber("Elevator Velocity (Meters per Second)", getVelocityMetersPerSecond());
//         SmartDashboard.putNumber("Elevator Applied Voltage", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
//         SmartDashboard.putNumber("Elevator Applied Output", m_motor.getAppliedOutput());
//     }
// }


