// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private static final CANdle candle = new CANdle(MechanismConstants.CANdleID);
  private static final CANdleConfiguration config = new CANdleConfiguration();
  private static final int numLEDS = 33 + 8;
  private static final Color zeroColor = new Color(0, 0, 0);

  private double time = 0.0;

  /** Creates a new Lights. */
  public Lights() {
    config.CANdleFeatures.withEnable5VRail(Enable5VRailValue.Disabled)
        .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled)
        .withVBatOutputMode(VBatOutputModeValue.On);
    config
        .LED
        .withBrightnessScalar(0.5)
        .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning)
        .withStripType(StripTypeValue.GRBW);

    candle.getConfigurator().apply(config);

    clearAllAnimations();
  }

  public void clearAllAnimations() {
    int max = candle.getMaxSimultaneousAnimationCount().getValue();
    for (int i = 0; i < max; i++) {
      candle.setControl(new EmptyAnimation(i));
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
        new StrobeAnimation(0, numLEDS)
            .withColor(new RGBWColor(r, g, b, Math.min(r, Math.min(g, b))))
            .withFrameRate(2);
    setLEDstrip(animation);
  }

  private void setStrobe(Color color) {
    setLEDstrip(new StrobeAnimation(0, numLEDS).withColor(new RGBWColor(color)).withFrameRate(1));
  }

  private void setFade(Color color) {
    setLEDstrip(
        new SingleFadeAnimation(0, numLEDS).withColor(new RGBWColor(color)).withFrameRate(10));
  }

  private void setFade(int r, int g, int b) {
    setLEDstrip(
        new SingleFadeAnimation(0, numLEDS)
            .withColor(new RGBWColor(r, g, b, Math.min(r, Math.min(g, b))))
            .withFrameRate(10));
  }

  public void setSolid(int r, int g, int b) {
    SolidColor animation =
        new SolidColor(0, numLEDS).withColor(new RGBWColor(r, g, b, Math.min(r, Math.min(g, b))));
    setLEDstrip(animation);
  }

  public void setSolid(Color color) {
    setLEDstrip(new SolidColor(0, numLEDS).withColor(new RGBWColor(color)));
  }

  public void setLEDstrip(ControlRequest animation) {
    candle.setControl(animation);
  }
}
