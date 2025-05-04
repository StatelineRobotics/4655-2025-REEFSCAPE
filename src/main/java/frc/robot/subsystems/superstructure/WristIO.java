package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

// Values to control subsystem
public interface WristIO {
  @AutoLog
  class WristIOInputs {

    public double wristPos = 0.0;

    public double wristDutyCycle = 0.0;
    public double wristAppliedVoltage = 0.0;
    public double wristAppliedCurrent = 0.0;
    public double wristSetpoint = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void requestWristPosition(double targetPos, double arbFeedforward) {}

  default void requestWristVoltage(double voltage) {}

  default void stopWrist() {}

  default void stop() {}
}
