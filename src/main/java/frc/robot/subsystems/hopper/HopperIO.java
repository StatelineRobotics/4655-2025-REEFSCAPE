package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public class HopperIOInputs {

    public double pivotAngle = 0.0;
    public double pivotSetpoint = 0.0;
    public double pivotMotorVoltage = 0.0;
    public double pivotMotorCurrent = 0.0;

    public double beltMotorVoltage = 0.0;
    public double beltMotorCurrent = 0.0;
  }

  default void updateInputs(HopperIOInputs inputs) {}

  default void requestBeltVoltage(double voltage) {}

  default void requestPivotAngle(double angle) {}

  default void stopBelt() {}

  default void stopWrist() {}
}
