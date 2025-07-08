// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ScorePositionSelector {

  private ArrayList<Binding> currentBindings = new ArrayList<Binding>();
  private ArrayList<Binding> allBindings = new ArrayList<Binding>();
  private Command defaultCommand;
  private Command currentCommand = Commands.run(() -> {}).withName("waiting for initial input");

  public ScorePositionSelector(Command defaultCommand) {
    this.defaultCommand = defaultCommand;
    currentCommand.schedule();
  }

  public ScorePositionSelector addBinding(Binding binding) {
    allBindings.add(binding);
    binding
        .trigger
        // .onTrue(
        //     Commands.deferredProxy(this::getDesiredCommand)
        //         .beforeStarting(Commands.runOnce(() -> addToPossibeList(binding)))
        //         .withName(binding.name))
        // .onFalse(
        //     Commands.deferredProxy(this::getDesiredCommand)
        //         .beforeStarting(Commands.runOnce(() -> removeFromPossibleList(binding))));
        .onTrue(
            Commands.deferredProxy(
                    () -> {
                      addToPossibeList(binding);
                      return getDesiredCommand();
                    })
                .withName(binding.name))
        .onFalse(
            Commands.deferredProxy(
                () -> {
                  removeFromPossibleList(binding);
                  return getDesiredCommand();
                }));

    return this;
  }

  private void addToPossibeList(Binding binding) {
    currentBindings.add(0, binding);
  }

  private void removeFromPossibleList(Binding binding) {
    for (int i = 0; i < currentBindings.size(); i++) {
      if (currentBindings.get(i) == binding) {
        currentBindings.remove(i);
        i--;
      }
    }
  }

  private Command getDesiredCommand() {
    currentCommand.cancel();
    if (currentBindings.size() == 0) {
      currentCommand = defaultCommand;
      return currentCommand;
    } else {
      Binding current = currentBindings.get(0);
      currentCommand = current.targetCommand;
      return currentCommand;
    }
  }

  public void log() {
    String[] names = new String[allBindings.size()];
    for (int i = 0; i < names.length; i++) {
      names[i] = allBindings.get(i).name;
    }

    String[] currentNames = new String[currentBindings.size()];
    for (int i = 0; i < currentNames.length; i++) {
      currentNames[i] = currentBindings.get(i).name;
    }

    Logger.recordOutput("Selector/AllBindings", names);
    Logger.recordOutput("Selector/CurrentBindings", currentNames);
    Logger.recordOutput("Selector/CurrentCommand", currentCommand.getName());
    SmartDashboard.putData("Selector.CurrentCommand", currentCommand);
  }
}
