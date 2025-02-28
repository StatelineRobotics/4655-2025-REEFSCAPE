// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;

/** Add your docs here. */
public class ScorePositionSelector {

  private Trigger alternateButton;
  private ArrayList<Binding> currentBindings = new ArrayList<Binding>();
  private ArrayList<Binding> allBindings = new ArrayList<Binding>();
  private Command defaultCommand;
  private Command currentCommand;
  private boolean wantUpdate = false;
  private Trigger needsUpdate =
      new Trigger(() -> wantUpdate)
          .onTrue(
              Commands.deferredProxy(
                  () ->
                      new InstantCommand(
                              () -> {
                                wantUpdate = false;
                              })
                          .andThen(currentCommand)));

  public ScorePositionSelector(Trigger alternateButton, Command defaultCommand) {
    this.alternateButton = alternateButton;
    this.defaultCommand = defaultCommand;
  }

  public ScorePositionSelector addBinding(Binding binding) {
    allBindings.add(binding);
    binding
        .trigger
        .onTrue(new InstantCommand(() -> addToPossibeList(binding)))
        .onFalse(new InstantCommand(() -> removeFromPossibleList(binding)));

    return this;
  }

  private void addToPossibeList(Binding binding) {
    currentBindings.add(0, binding);
    updateCurrentCommand();
  }

  private void removeFromPossibleList(Binding binding) {
    for (int i = 0; i < currentBindings.size(); i++) {
      if (currentBindings.get(i) == binding) {
        currentBindings.remove(i);
        i--;
      }
    }
    updateCurrentCommand();
  }

  private void updateCurrentCommand() {
    if (currentBindings.size() == 0) {
      currentCommand = defaultCommand;
      wantUpdate = true;
    } else {
      Binding current = currentBindings.get(0);
      currentCommand =
          new ConditionalCommand(current.alternateCommand, current.targetCommand, alternateButton);
      wantUpdate = true;
    }
  }
}
