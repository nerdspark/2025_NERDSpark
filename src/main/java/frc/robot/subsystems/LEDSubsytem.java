// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

public class LEDSubsytem extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;


  public LEDSubsytem() {
    m_led = new AddressableLED(LEDConstants.kPort);
    m_buffer = new AddressableLEDBuffer(LEDConstants.kLength);
    m_led.setLength(LEDConstants.kLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display


    m_led.setData(m_buffer);
  }

  public LEDPattern getPattern(Trigger driveTrainFinishedMoving, Trigger bucketHasCoral, Trigger gripperHasGamePiece){
    boolean hasGamePiece = bucketHasCoral.getAsBoolean() || gripperHasGamePiece.getAsBoolean();
    Color color = (hasGamePiece) ? new Color(0, 1, 0) : new Color(0, 0, 1); 
    boolean flashing = hasGamePiece ? driveTrainFinishedMoving.getAsBoolean() : false;
    if (flashing) {
      return LEDPattern.solid(color).blink(Time.ofBaseUnits(0.05, Second));
    } else {
      return LEDPattern.solid(color);
    }
  }
  public Color[] updateStepColor(Trigger armFinishedMoving, Trigger driveTrainFinishedMoving, Trigger hasCoral) {
    Color step1 = new Color();
    Color step2 = new Color();
    Color step3 = new Color();

    
      if(armFinishedMoving.getAsBoolean()) { // 
        step1 = new Color(1.0f, 1.0f, 0.0f); // yellow
      } else {
        step1 = new Color(1.0f, 0.0f, 1.0f); // cyan
      }

      if(driveTrainFinishedMoving.getAsBoolean()) { // 
        step2 = new Color(0.0f, 1.0f, 1.0f); // magenta
      } else {
        step2 = new Color(0.0f, 0.0f, 1.0f); // blue
      }

      if (hasCoral.getAsBoolean()) { // 
        step3 =  new Color(1.0f, 0.0f, 0.0f); // green
      } else {
        step3 = new Color(0.0f, 1.0f, 0.0f);  // red

      } 

      Color[] colors = {step1, step2, step3};
      return colors;
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