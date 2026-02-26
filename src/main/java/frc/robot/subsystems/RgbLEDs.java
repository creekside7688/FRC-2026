// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.annotation.Retention;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

public class RgbLEDs extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  final Distance kLedSpacing = Meters.of(4 / 60.0);
  final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(4), kLedSpacing);
  final LEDPattern black = LEDPattern.solid(Color.kBlack);


  /**
   * Creates a new RGBLEDs.
   * 
   * @return
   */
  public RgbLEDs() {
    led = new AddressableLED(8);
    ledBuffer = new AddressableLEDBuffer(90);

    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();

    // RgbSolidRed();
    setDefaultCommand(runPattern(black).withName("Off"));
  }

  public Command RgbSolidRed() {
    LEDPattern red = LEDPattern.solid(Color.kRed);
    return runPattern(red);
  }

  public Command rgbRainbow() {
    return runPattern(scrollingRainbow);
  }
  
  public Command colorGradient(Color color1, Color color2) {
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2);
    return runPattern(gradient);
  }

  public Command colorBreathe(Color color1, Color color2) {
  LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2);
  LEDPattern breathe = base.breathe(Seconds.of(1));
  return runPattern(breathe);
  }

  public Command colorScroll(Color color1, Color color2) {
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2);
    LEDPattern scroll = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    return runPattern(scroll);
  }

  public Command rgbSolid(int r, int g, int b) {
    LEDPattern color = LEDPattern.solid(new Color(r, g, b));
    return runPattern(color);
  }

  public Command ledBlink(LEDPattern pattern, double delay) {
    LEDPattern blink = pattern.blink(Seconds.of(delay));
    return runPattern(blink);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(ledBuffer));
  }

  public void RGBflash() {
    LEDPattern white = LEDPattern.solid(Color.kWhite);
    LEDPattern pattern = white.blink(Seconds.of(1.5));
    pattern = pattern.atBrightness(Percent.of(200));
    runPattern(pattern);
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }
}