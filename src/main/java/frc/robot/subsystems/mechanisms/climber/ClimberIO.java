package frc.robot.subsystems.mechanisms.climber;

import org.littletonrobotics.junction.AutoLog;

// Values to control subsystem
public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {

    public double climberPOS = 0.0;
    public double funnelPOS = 0.0;
    public double climberCurrent = 0.0;
  }

  default void voltageControl(double voltage) {}

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setClimberPosition(double position) {}

  default void requestPull() {}

  default void stop() {}
}
