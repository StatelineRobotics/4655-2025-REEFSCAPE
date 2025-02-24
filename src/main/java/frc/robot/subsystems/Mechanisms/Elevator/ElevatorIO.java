package frc.robot.subsystems.mechanisms.elevator;

import org.littletonrobotics.junction.AutoLog;

// Values to control subsystem
public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {

    public boolean zeroed = false;

    public double elevatorPos = 0.0;
    public double elevatorVelo = 0.0;
    public double dutyCycle = 0.0;
    public double appliedVolts = 0.0;
    public double appliedCurrent = 0.0;
    public double setPoint = 0.0;

  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void requestElevatorPosition(){}

  default void requestFunnelPOS() {}

  default void voltageControl(double voltage) {}

  default void positionControl(double target) {}

  default void requestBeltRPM() {}

  default void setElevatorPosition(double position) {}

  default void stop() {}
}
