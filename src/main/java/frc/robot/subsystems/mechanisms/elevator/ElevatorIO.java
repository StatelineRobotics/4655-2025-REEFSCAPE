package frc.robot.subsystems.mechanisms.elevator;

import org.littletonrobotics.junction.AutoLog;

// Values to control subsystem
public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {

    public boolean zeroed = false;

    public double elevatorPos = 0.0;
    public double funnelPos = 0.0;
    public double funnelVoltage = 0.0;
    public double elevatorVelo = 0.0;
    public double dutyCycle = 0.0;
    public double appliedVolts = 0.0;
    public double appliedCurrent = 0.0;
    public double finalSetpoint = 0.0;
    public double motionSetpoint = 0.0;
    public double veolocitySetpoint = 0.0;
    public double setpoint = 0.0;
    public double funnelCurrent = 0.0;

    public boolean isAtBottom = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void requestFunnelPOS(double postiion) {}

  default void voltageControl(double voltage) {}

  default void requestElevatorPosition(double targetPostion, double feedforward) {}

  default void requestBeltRPM() {}

  default void reqestBeltVoltage(double voltage) {}

  default void setElevatorPosition(double position) {}

  default void stop() {}
}
