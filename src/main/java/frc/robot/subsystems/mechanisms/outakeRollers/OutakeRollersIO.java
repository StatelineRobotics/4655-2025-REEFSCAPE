package frc.robot.subsystems.mechanisms.outakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface OutakeRollersIO {
  @AutoLog
  public class OutakeRollersIOInputs {

    public double leftMotorVoltage = 0.0;
    public double leftMotorVelocity = 0.0;
    public double leftMotorCurrent = 0.0;

    public double rightMotorVoltage = 0.0;
    public double rightMotorVelocity = 0.0;
    public double rightMotorCurrent = 0.0;

    public boolean manipulatorLaserDetects = false;
    public boolean elevatorLaserDetects = false;
  }

  default void updateInputs(OutakeRollersIOInputs inputs) {}

  default void requestOutakeVoltage(double voltage) {}

  default void stopOutake() {}
}
