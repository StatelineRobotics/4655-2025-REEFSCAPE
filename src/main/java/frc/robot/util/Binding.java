// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Binding {
  Trigger trigger;
  Command targetCommand;
  Command alternateCommand;

  public Binding(Trigger trigger, Command targetCommand, Command alternateCommand) {
    this.trigger = trigger;
    this.targetCommand = targetCommand;
    this.alternateCommand = alternateCommand;
  }
}
