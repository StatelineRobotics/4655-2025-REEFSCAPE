// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {
  private static final CANdle candle = new CANdle(MechanismConstants.CANdleID);
  private static final CANdleConfiguration config = new CANdleConfiguration();
  private static final int numLEDS = 33 + 8;
  private static final Color zeroColor = new Color(0, 0, 0);

  private double time = 0.0;

  /** Creates a new Lights. */
  public Lights() {
    config.brightnessScalar = 0.5;
    config.disableWhenLOS = false;
    config.statusLedOffWhenActive = false;
    config.stripType = LEDStripType.GRBW;
    candle.configAllSettings(config);
    clearAllAnimations();
    setFade(new Color(80, 7, 120));
    // candle.animate(getFadeAnimation(255, 209, 0), 1);
  }

  public void clearAllAnimations() {
    int max = candle.getMaxSimultaneousAnimationCount();
    for (int i = 0; i < max; i++) {
      candle.animate(null, i);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LIGHTS/TIME", time);
  }

  private double minValue(Color color) {
    return Math.min(color.blue, Math.min(color.red, color.blue));
  }

  private void setStrobe(int r, int g, int b) {
    StrobeAnimation animation =
        new StrobeAnimation(r, g, b, Math.min(r, Math.min(g, b)), 0.5, numLEDS);
    Logger.recordOutput("Lights/latestOutput", "strobe: " + r + ", " + g + ", " + b);
    setLEDstrip(animation);
  }

  private void setStrobe(Color color) {
    setStrobe((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  public Command strobeAnimation(Color color, String name) {
    return this.startRun(() -> setStrobe(color), () -> {}).withName(name);
  }

  public Command strobeAnimation(Color color, BooleanSupplier condition, String name) {
    return this.startRun(() -> setStrobe(color), () -> {}).until(condition).withName(name);
  }

  private void setFade(Color color) {
    setFade((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  private void setFade(int r, int g, int b) {
    SingleFadeAnimation animation =
        new SingleFadeAnimation(r, g, b, Math.min(r, Math.min(g, b)), 0.5, numLEDS);
    Logger.recordOutput("Lights/latestOutput", "fade: " + r + ", " + g + ", " + b);
    setLEDstrip(animation);
  }

  public Command fadeAnimation(Color color, String name) {
    return this.startRun(() -> setFade(color), () -> {}).withName(name);
  }

  public Command fadeAnimation(Color color, BooleanSupplier condition, String name) {
    return this.startRun(() -> setFade(color), () -> {}).until(condition).withName(name);
  }

  public void setSolid(int r, int g, int b) {
    Logger.recordOutput("Lights/latestOutput", "solid: " + r + ", " + g + ", " + b);
    candle.animate(null, 0);
    candle.setLEDs(r, g, b, Math.min(r, Math.min(g, b)), 0, numLEDS);
  }

  public void setSolid(Color color) {
    setSolid((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  public Command solidAnimation(Color color, String name) {
    return this.startRun(() -> setSolid(color), () -> {}).withName(name);
  }

  public Command solidAnimation(Color color, BooleanSupplier condition, String name) {
    return this.startRun(() -> setSolid(color), () -> {}).until(condition).withName(name);
  }

  public void setLEDstrip(Animation animation) {
    animation.setLedOffset(0);
    animation.setNumLed(numLEDS);
    candle.animate(animation, 0);
  }
}
