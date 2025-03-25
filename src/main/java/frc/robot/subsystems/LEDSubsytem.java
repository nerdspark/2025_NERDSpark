// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class LEDSubsytem extends SubsystemBase {
  private static final int kPort = Constants.LEDConstants.kPort;
  private static final int kLength = Constants.LEDConstants.kLength;
  private Supplier<LEDPattern> currentPattern = () -> LEDPattern.solid(new Color(1.0f, 1.0f ,1.0f));

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


  public Color getColor(Supplier<String> level){
    // boolean hasGamePiece = bucketHasCoral.get() || gripperHasGamePiece.get();
    Color color = new Color();
    // boolean flashing = hasGamePiece ? driveTrainFinishedMoving.get() : false;

    // if (l1.get() == true && l2.get() == false && l3.get() == false && l4.get() == false) {
    //   color = new Color(0.0f, 1.0f, 0.0f); // red
    // } else if (l1.get() == false && l2.get() == true && l3.get() == false && l4.get() == false) {
    //   color = new Color(0.0f, 1.0f, 1.0f); // magenta
    // } else if (l1.get() == false && l2.get() == false && l3.get() == true && l4.get() == false) {
    //   color = new Color(0.0f, 0.0f, 1.0f); // blue
    // } else if (l1.get() == false && l2.get() == false && l3.get() == false && l4.get() == true) {
    //   color = new Color(1.0f, 0.0f, 0.0f); // green
    // }

    switch (level.get()) {
      case "l1":
        color = new Color(0.0f, 1.0f, 0.0f);  // red
        break;
      case "l2":
        color = new Color(0.0f, 1.0f, 1.0f); // magenta
        break;
      case "l3":
        color = new Color(0.0f, 0.0f, 1.0f); // blue
        break;
      case "l4":
        color = new Color(1.0f, 0.0f, 0.0f); // green
        break;
      default:
        color = new Color(1.0f, 1.0f, 1.0f); // default color: this case will never happen
    }


    return color;

    

  }
  public Color[] updateStepColor(Trigger armFinishedMoving, Trigger driveTrainFinishedMoving, Trigger hasCoral) {
    Color step1 = new Color();
    Color step2 = new Color();

      if(hasCoral.getAsBoolean()) { // 
        step2 = new Color(1.0f, 0.0f, 0.0f); // cyan
      } else {
        step2 = new Color(0.0f, 1.0f, 0.0f); // red
      }

      // if (detectedCoral.get()) { // 
      //   step3 =  new Color(1.0f, 0.0f, 0.0f); // green
      // } else {
      //   step3 = new Color(0.0f, 1.0f, 0.0f);  // red
      // } 

      Color[] colors = {step2};
      return colors;
// >>>>>>> develop_led4
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
    // currentPattern = pattern;
    return run(() -> pattern.get().applyTo(m_buffer));
  }
  public Command blink() {
    return new InstantCommand(() -> currentPattern.get().blink(Seconds.of(0.5)).applyTo(m_buffer));
  
  public Command breathe() {
    return new InstantCommand(() -> currentPattern.get().breathe(Seconds.of(2.0)).applyTo(m_buffer));
  }
}



// distance between two poses -> ratio of gradient joystick.map()
