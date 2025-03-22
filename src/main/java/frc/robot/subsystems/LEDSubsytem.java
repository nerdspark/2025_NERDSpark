// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.BooleanSupplier;
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

public class LEDSubsytem extends SubsystemBase {
  private static final int kPort = Constants.LEDConstants.kPort;
  private static final int kLength = Constants.LEDConstants.kLength;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;


  public LEDSubsytem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display


    m_led.setData(m_buffer);
  }

  public Color getPattern(BooleanSupplier driveTrainFinishedMoving, BooleanSupplier bucketHasCoral, BooleanSupplier gripperHasGamePiece){
    boolean hasGamePiece = bucketHasCoral.getAsBoolean() || gripperHasGamePiece.getAsBoolean();
    Color color = (hasGamePiece) ? new Color(0.5f, 0.0f, 0.0f) : new Color(1.0f, 0.0f, 0.0f); // GRB
    boolean flashing = hasGamePiece ? driveTrainFinishedMoving.getAsBoolean() : false;
    // if (flashing) {
    //   return LEDPattern.solid(color).blink(Time.ofBaseUnits(0.05, Second));
    // } else {
    //   return LEDPattern.solid(color);
    // }
    return color;
  }
  // public Color[] updateStepColor(Trigger armFinishedMoving, Trigger driveTrainFinishedMoving, Trigger hasCoral) {
  //   Color step1 = new Color();
  //   Color step2 = new Color();

  //     if(hasCoral.getAsBoolean()) { // 
  //       step2 = new Color(1.0f, 0.0f, 0.0f); // cyan
  //     } else {
  //       step2 = new Color(0.0f, 1.0f, 0.0f); // red
  //     }

  //     // if (detectedCoral.get()) { // 
  //     //   step3 =  new Color(1.0f, 0.0f, 0.0f); // green
  //     // } else {
  //     //   step3 = new Color(0.0f, 1.0f, 0.0f);  // red
  //     // } 

  //     Color[] colors = {step2};
  //     return colors;
  // }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(Supplier<LEDPattern> pattern) {
    return run(() -> pattern.get().applyTo(m_buffer));
  }
}



// distance between two poses -> ratio of gradient joystick.map()
