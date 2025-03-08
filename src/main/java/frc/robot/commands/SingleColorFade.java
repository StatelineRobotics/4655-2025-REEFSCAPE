// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SingleColorFade extends Command {
  private Color targetColor;
  private Color noColor = new Color();
  private Lights lights;
  private double time;
  private boolean increasing = true;
  /** Creates a new SingleColorFade. */
  public SingleColorFade(Color targetColor, Lights lights) {
    this.targetColor = targetColor;
    this.lights = lights;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0.0;
    increasing = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (increasing) {
      time += .02;
      if (time >= 1.0) {
        increasing = false;
      }
    } else {
      time -= .02;
    }

    lights.setSolidColor(Color.lerpRGB(noColor, targetColor, time));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!increasing && time <= 0.0) {
      return true;
    }
    return false;
  }
}
