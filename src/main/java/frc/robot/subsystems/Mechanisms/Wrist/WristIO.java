package frc.robot.subsystems.mechanisms.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

// Values to control subsystem
public interface WristIO {
  @AutoLog
  class WristIOInputs {

    public double wristPos = 0.0;
    public double leftIntakeRPM = 0.0;
    public double rightIntakeRPM = 0.0;
    public double funnelRPM = 0.0;

    public double wristDutyCycle = 0.0;
    public double wristAppliedVoltage = 0.0;
    public double wristAppliedCurrent = 0.0;
    public double wristSetpoint = 0.0;

    public double rightDutyCycle = 0.0;
    public double rightAppliedVoltage = 0.0;
    public double rightAppliedCurrent = 0.0;
    public double rightSetpoint = 0.0;

    public double leftDutyCycle = 0.0;
    public double leftAppliedVoltage = 0.0;
    public double leftAppliedCurrent = 0.0;
    public double leftSetpoint = 0.0;
  }

  default void updateInputs(WristIOInputs inputs){}

  default void requestIntakeVelo(double targetVelo){}

  default void requestIntakeVoltage(double voltage){}

  default void requestWristPosition(double targetPos){}

  default void requestWristVoltage(double voltage){}

  default void stopIntake(){}

  default void stopWrist(){}

  default void stop(){}
}
