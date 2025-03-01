// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Binding {
  public Trigger trigger;
  public Command targetCommand;
  public String name;

  public Binding(Trigger trigger, Command targetCommand) {
    this.trigger = trigger;
    this.targetCommand = targetCommand;
    this.name = targetCommand.getName();
  }

  public Binding(Trigger trigger, Command targetCommand, String name) {
    this.trigger = trigger;
    this.targetCommand = targetCommand;
    this.name = name;
  }
}
