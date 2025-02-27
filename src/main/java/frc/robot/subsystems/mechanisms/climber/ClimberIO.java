package frc.robot.subsystems.mechanisms.climber;

import org.littletonrobotics.junction.AutoLog;

// Values to control subsystem
public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {

    public double climberPOS = 0.0;
    public double funnelPOS = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setClimberPosition() {}

  default void setFunnelPosition() {}

  default void stop() {}
}
