package frc.robot.subsystems.mechanisms.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO io;

  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private static final double pivotAllowedError = 1.0;
  private static final double intakeAngle = 0.0;
  private static final double climbAngle = 100.0;

  private static final double intakeBeltVoltage = 6.0;

  public Trigger pivotAtSetpoint = new Trigger(this::isPivotAtSetpoint);

  /** Creates a new ExampleSubsystem. */
  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private boolean isPivotAtSetpoint() {
    return Math.abs(inputs.pivotSetpoint - inputs.pivotAngle) > pivotAllowedError;
  }

  // Call to drive pivot to a setpoint
  private void requestPivotAngle(double angle) {
    inputs.pivotSetpoint = angle;
    io.requestPivotAngle(angle);
  }

  // Call to drive belt at requested voltage
  private void requestBeltVoltage(double voltage) {
    io.requestBeltVoltage(voltage);
  }

  private void stopBelt() {
    io.stopBelt();
  }

  public Command intakeCommand() {
    return (run(() -> requestPivotAngle(intakeAngle))
            .until(pivotAtSetpoint)
            .withName("Pivot To Intake"))
        .andThen(runEnd(() -> requestBeltVoltage(8.0), this::stopBelt).withName("Run Belt Intake"))
        .withName("Inake Sequence");
  }

  public Command climbComand() {
    return run(() -> requestPivotAngle(climbAngle));
  }
}
