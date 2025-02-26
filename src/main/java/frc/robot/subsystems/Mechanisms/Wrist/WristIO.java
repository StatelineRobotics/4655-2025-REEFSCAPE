package frc.robot.subsystems.mechanisms.wrist;

import org.littletonrobotics.junction.AutoLog;

// Values to control subsystem
public interface WristIO {
  @AutoLog
  class WristIOInputs {

    public double wristPos = 0.0;
    public double leftIntakeRPM = 0.0;
    public double rightIntakeRPM = 0.0;
    public double funnelRPM = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void requestIntakeVelo(double targetVelo) {}

  default void requestIntakeVoltage(double voltage) {}

  default void requestWristPosition(double targetPos) {}

  default void requestWristVoltage(double voltage) {}

  default void stopIntake() {}

  default void stopWrist() {}

  default void stop() {}
}
