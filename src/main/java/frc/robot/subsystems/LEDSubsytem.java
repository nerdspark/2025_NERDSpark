// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class LEDSubsytem extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  private Trigger armFinishedMoving;
  private Trigger driveTrainFinishedMoving;
  private Trigger hasCoral;
  public Arm arm;
  private Intake intake;
  public ScoringProfileSubsystem scoringSubsystem;
  public PoseEstimatorSubsystem poseEstimatorSubsystem;


  public LEDSubsytem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    armFinishedMoving = new Trigger(() -> arm.finishedMoving);
    driveTrainFinishedMoving = new Trigger (() -> poseEstimatorSubsystem.getCurrentPose().getTranslation()
    .getDistance(scoringSubsystem.getSelectedBranchPose().getTranslation()) < 1 || poseEstimatorSubsystem.getCurrentPose().getTranslation()
    .getDistance((scoringSubsystem.getSelectedCoralStationPose().getTranslation()))<1);
    hasCoral = new Trigger(() -> intake.hasCoral());
    

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    //setDefaultCommand(runPattern(LEDPattern.solid(new Color(0.0f, 0.0f, 1.0f))));
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display


    m_led.setData(m_buffer);
    getStepColor(armFinishedMoving, driveTrainFinishedMoving);
  }

  public void getStepColor(Trigger armFinishedMoving, Trigger driveTrainFinishedMoving) {
    this.armFinishedMoving = armFinishedMoving;
    this.driveTrainFinishedMoving = driveTrainFinishedMoving;
    Color step1 = new Color();
    Color step2 = new Color();
    Color step3 = new Color();

    
      if(armFinishedMoving.getAsBoolean()) {
        step1 = new Color(0.0f, 0.0f, 1.0f); // blue
      } else {
        step1 = new Color(1.0f, 0.0f, 0.0f); // green
      }
      if(driveTrainFinishedMoving.getAsBoolean()) {
        step2 = new Color(0.0f, 1.0f, 0.0f); // red
      } else {
        step2 = new Color(1.0f, 1.0f, 0.0f); // yellow
      }
      if (hasCoral.getAsBoolean()) {
        // step3 =  new Color(0.0f, 0.0f, 1.0f); 
      } else {
        // step3 = new Color(1.0f, 1.0f, 0.0f); 

      } 
      runPattern(LEDPattern.steps(Map.of(0, step1, 0.5, step2))
        .scrollAtRelativeSpeed(Percent.per(Second).of(Constants.scrollSpeed)));
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }
}