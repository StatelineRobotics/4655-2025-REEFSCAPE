package frc.robot.subsystems.Mechanisms.Climber;

import org.littletonrobotics.junction.AutoLog;

// Values to control subsystem
public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {

    public double ClimberPos = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void stop() {}
}
