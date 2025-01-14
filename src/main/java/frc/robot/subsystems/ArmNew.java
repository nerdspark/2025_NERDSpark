package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.nio.file.ProviderMismatchException;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmGains;
import frc.robot.Constants.ArmConstants.ArmSetPoints;
import frc.robot.util.LightningShuffleboard;
import frc.robot.Constants.RobotMap;

public class ArmNew {
    private TalonFX shoulderRight;
    private TalonFX shoulderLeft;
    private TalonFX elbowLeft;
    private TalonFX elbowRight;
    private TalonFX wrist1;
    private TalonFX wrist2;
    private CANSparkMax gripper;

    private RelativeEncoder gripperEncoder;

    // private PIDController shoulderLeftController;
    // private PIDController shoulderRightController;
    // private PIDController elbowLeftController;
    // private PIDController elbowRightController;

    private boolean inBend;
    // private PIDController shoulderController;
    // private PIDController elbowController;
    // private ArmFeedforward shoulderLeftFeedforward;
    // private ArmFeedforward shoulderRightFeedforward;
    // private ArmFeedforward elbowLeftFeedforward;
    // private ArmFeedforward elbowRightFeedforward;

    private ArmGains armGains = new ArmGains();


    public ArmNew() {

        shoulderLeft = new TalonFX(RobotMap.shoulderLeftID, "canivore1");
        shoulderRight = new TalonFX(RobotMap.shoulderRightID, "canivore1");
        elbowLeft = new TalonFX(RobotMap.elbowLeftID, "canivore1");
        elbowRight = new TalonFX(RobotMap.elbowRightID, "canivore1");

        wrist1 = new TalonFX(2, "canivore1"); //TODO placeholder value CHANGE THIS WHEN BUILT
        wrist2 = new TalonFX(3, "canivore1"); //TODO placeholder value CHANGE THIS WHEN BUILT

        TalonFXConfiguration shoulderconfig = new TalonFXConfiguration();
        TalonFXConfiguration elbowconfig = new TalonFXConfiguration();

        TalonFXConfiguration wrist1Config; //TODO write this in later
        TalonFXConfiguration wrist2Config; //TODO write this in later

        shoulderconfig.CurrentLimits =
                new CurrentLimitsConfigs().withStatorCurrentLimit(ArmConstants.currentLimitShoulder).withStatorCurrentLimitEnable(true);
        shoulderconfig.Feedback = new FeedbackConfigs()
                .withFeedbackRotorOffset(0.0)//ArmConstants.shoulderOffset)
                .withSensorToMechanismRatio(ArmConstants.shoulderRadPerRot);
        shoulderconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.3 );
        shoulderconfig.Slot0 = new Slot0Configs()
                .withKP(armGains.shoulderP)
                .withKI(armGains.shoulderI)
                .withKD(armGains.shoulderD)
                .withKG(armGains.shoulderG)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        shoulderLeft
                .getConfigurator()
                .apply(shoulderconfig.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));
        shoulderRight
                .getConfigurator()
                .apply(shoulderconfig.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));

        elbowconfig.CurrentLimits =
                new CurrentLimitsConfigs().withStatorCurrentLimit(ArmConstants.currentLimitElbow).withStatorCurrentLimitEnable(true);
        elbowconfig.Feedback = new FeedbackConfigs()
                .withFeedbackRotorOffset(0.0)//ArmConstants.elbowOffset)
                .withSensorToMechanismRatio(ArmConstants.elbowRadPerRot);
        elbowconfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);
        elbowconfig.Slot0 = new Slot0Configs()
                .withKP(armGains.elbowP)
                .withKI(armGains.elbowI)
                .withKD(armGains.elbowD)
                .withKG(armGains.elbowG)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        elbowLeft
                .getConfigurator()
                .apply(elbowconfig.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));
        elbowRight
                .getConfigurator()
                .apply(elbowconfig.withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)));

        gripper = new CANSparkMax(RobotMap.gripperID, MotorType.kBrushless);

        // shoulderLeft.setNeutralMode(NeutralModeValue.Coast);
        // shoulderRight.setNeutralMode(NeutralModeValue.Coast);
        // elbowLeft.setNeutralMode(NeutralModeValue.Coast);
        // elbowRight.setNeutralMode(NeutralModeValue.Coast);

        // shoulderLeft.setInverted(true);
        // shoulderRight.setInverted(false);
        // elbowRight.setInverted(false);
        // elbowLeft.setInverted(true);
        gripper.setInverted(true);

        gripperEncoder = gripper.getEncoder();

        inBend = false;
        resetEncoders();
        // setArmGains(climbGains);

        // shoulderLeftEncoder.setPosition(ArmConstants.shoulderOffset);
        // shoulderRightEncoder.setPosition(ArmConstants.shoulderOffset);
        // elbowLeftEncoder.setPosition(ArmConstants.elbowOffset);
        // elbowRightEncoder.setPosition(ArmConstants.elbowOffset);
        // gripperEncoder.setPosition(0);

    }


    // public void setArmVelocity(Translation2d velocity) {
    //     setArmPosition(getArmPosition().plus(velocity), inBend);
    // }

    // private void resetArmGains(ArmGains gains) {
    //     shoulderLeftController.setPID(gains.shoulderP, gains.shoulderI, gains.shoulderD);
    //     shoulderRightController.setPID(gains.shoulderP, gains.shoulderI, gains.shoulderD);
    //     elbowLeftController.setPID(gains.elbowP, gains.elbowI, gains.elbowD);
    //     elbowRightController.setPID(gains.elbowP, gains.elbowI, gains.elbowD);

    //     shoulderLeftFeedforward = new ArmFeedforward(
    //             gains.shoulderS, gains.shoulderG, gains.shoulderV, gains.shoulderA);
    //     shoulderRightFeedforward = new ArmFeedforward(
    //             gains.shoulderS, gains.shoulderG, gains.shoulderV, gains.shoulderA);
    //     elbowLeftFeedforward =
    //             new ArmFeedforward(gains.elbowS, gains.elbowG, gains.elbowV, gains.elbowA);
    //     elbowRightFeedforward =
    //             new ArmFeedforward(gains.elbowS, gains.elbowG, gains.elbowV, gains.elbowA);

    //     elbowLeftController.setIZone(0.15);
    //     elbowRightController.setIZone(0.15);
    // }
    public void setArmPosition(Translation2d position, boolean inBend) {
        this.inBend = inBend;
        double distance = MathUtil.clamp(position.getNorm(), ArmSetPoints.home.getNorm(), ArmConstants.baseStageLength + ArmConstants.secondStageLength);

        double BaseAngleArmDiff = Math.acos(((distance * distance)
                        + (ArmConstants.baseStageLength * ArmConstants.baseStageLength)
                        - (ArmConstants.secondStageLength * ArmConstants.secondStageLength))
                / (2 * distance * ArmConstants.baseStageLength));
        double SecondAngleArmDiff = Math.acos(((distance * distance)
                        - (ArmConstants.baseStageLength * ArmConstants.baseStageLength)
                        + (ArmConstants.secondStageLength * ArmConstants.secondStageLength))
                / (2 * distance * ArmConstants.secondStageLength));

        double shoulderPosition = position.getAngle().getRadians() + (BaseAngleArmDiff * (inBend ? 1 : -1));
        double elbowPosition = position.getAngle().getRadians() + (SecondAngleArmDiff * (inBend ? -1 : 1));
        setShoulderPosition(shoulderPosition);
        setElbowPosition(elbowPosition);
        SmartDashboard.putNumber("shouldertarget", shoulderPosition);
        SmartDashboard.putNumber("elbowtarget", elbowPosition);
        SmartDashboard.putNumber("shoulderPosition error", shoulderPosition - getShoulderLeftPosition());
        SmartDashboard.putNumber("elbow R Position error", elbowPosition - getElbowRightPosition());
        SmartDashboard.putNumber("elbow L Position error", elbowPosition - getElbowLeftPosition());
    }

    public void resetEncoders() {
        elbowLeft.setPosition(ArmConstants.elbowOffset / (2.0*Math.PI));
        elbowRight.setPosition(ArmConstants.elbowOffset / (2.0*Math.PI));
        shoulderLeft.setPosition(ArmConstants.shoulderOffset / (2.0*Math.PI));
        shoulderRight.setPosition(ArmConstants.shoulderOffset / (2.0*Math.PI));

        gripperEncoder.setPosition(0);
    }

    public Translation2d getArmPosition() {
        Translation2d jointPos = new Translation2d(
                Math.cos(getShoulderLeftPosition()) * ArmConstants.baseStageLength,
                Math.sin(getShoulderLeftPosition()) * ArmConstants.baseStageLength);
        Translation2d jointToEndPos = new Translation2d(
                Math.cos(getElbowLeftPosition()) * ArmConstants.secondStageLength,
                Math.sin(getElbowLeftPosition()) * ArmConstants.secondStageLength);
        SmartDashboard.putNumber("arm x position", jointPos.plus(jointToEndPos).getX());
        SmartDashboard.putNumber("arm y position", jointPos.plus(jointToEndPos).getY());
        return jointPos.plus(jointToEndPos);
    }

    public void setShoulderPosition(double position) {
        position /= (2d * Math.PI);

        position = MathUtil.clamp(position, -0.1, 2.5);

        SmartDashboard.putNumber("shoulder position set raw", position);
        shoulderLeft.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));
        shoulderRight.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));
    }

    public double getShoulderLeftPosition() {
        SmartDashboard.putNumber("shoulder l position raw", shoulderLeft.getPosition().getValueAsDouble());
        return shoulderLeft.getPosition().getValueAsDouble() * (2d * Math.PI);
    }

    public double getShoulderRightPosition() {
        SmartDashboard.putNumber("shoulder r position raw", shoulderRight.getPosition().getValueAsDouble());
        return shoulderRight.getPosition().getValueAsDouble() * (2d * Math.PI);
    }

    public void setElbowPosition(double position) {
        // position -= getShoulderLeftPosition() * (1.0 - ArmConstants.virtual4BarGearRatio);
        position /= (2d * Math.PI);

        SmartDashboard.putNumber("elbow position set raw", position);
        elbowLeft.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));
        elbowRight.setControl(new PositionVoltage(position).withFeedForward(position).withPosition(position));
    }

    public double getElbowLeftPosition() {
        double elbowPose = elbowLeft.getPosition().getValueAsDouble() * (2d * Math.PI);
        // elbowPose += getShoulderLeftPosition() * (1.0 - ArmConstants.virtual4BarGearRatio);
        // SmartDashboard.putNumber("elbow l position", elbowPose);
        SmartDashboard.putNumber("elbow l position raw", elbowLeft.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeft.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeft.getPosition());
        return elbowPose;
        //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    public double getElbowRightPosition() {
        double elbowPose = elbowRight.getPosition().getValueAsDouble() * (2d * Math.PI);
        // elbowPose += getShoulderRightPosition() * (1.0 - ArmConstants.virtual4BarGearRatio);
        // SmartDashboard.putNumber("elbow r position", elbowPose);
        SmartDashboard.putNumber(
                "elbow r position raw", elbowRight.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("elbow adjustment factor", shoulderLeft.getPosition()*24.0/42.0);
        // SmartDashboard.putNumber("elbow to shoulder", elbowPose - shoulderLeft.getPosition());
        return elbowPose;
        //          + ((ArmConstants.virtual4BarGearRatio - 1) * (getShoulderPosition() - ArmConstants.shoulderOffset));
    }

    public void setGripperPower(double power) {
        gripper.set(power);
    }

    public double getGripperPosition() {
        return gripperEncoder.getPosition();
    }

    public double getElbowLeftVelocity() {
        return elbowLeft.getVelocity().getValueAsDouble();
    }

    public double getelbowLeftVelocity() {
        return shoulderLeft.getVelocity().getValueAsDouble();
    }

    public double getElbowRightVelocity() {
        return elbowRight.getVelocity().getValueAsDouble();
    }

    public double getShoulderRightVelocity() {
        return shoulderRight.getVelocity().getValueAsDouble();
    }
}
