package frc.robot.subsystems.Mechanisms.Elevator;

import org.littletonrobotics.junction.AutoLog;

// Values to control subsystem
public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {

    public double elevatorPos = 0.0;
    public double rightElevatorPosition = 0.0;
    public double leftElevatorPosition = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void stop() {}
}
