package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.FieldConstants.PieceType;

import org.littletonrobotics.junction.Logger;

public class OutakeRollers extends SubsystemBase {

  private final OutakeRollersIO io;
  private final OutakeRollersIOInputsAutoLogged inputs = new OutakeRollersIOInputsAutoLogged();

  public Trigger manipulatorDetects =
      new Trigger(() -> inputs.manipulatorLaserDetects).debounce(.25, DebounceType.kFalling);
  public Trigger elevatorDetects = new Trigger(() -> inputs.elevatorLaserDetects);
  public Trigger outakeStalled = new Trigger(this::motorsStalled);
  public Trigger hasAlgae = outakeStalled.debounce(.5, DebounceType.kBoth);

  /** Creates a new ExampleSubsystem. */
  public OutakeRollers(OutakeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("OutakeRollers", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void requestVoltageControl(double voltage) {
    io.requestOutakeVoltage(voltage);
  }

  private void stop() {
    io.stopOutake();
  }

  public Command stopRollers() {
    return runOnce(this::stop);
  }

  public Command score(double speed) {
    return run(() -> requestVoltageControl(speed))
        .until(manipulatorDetects.negate().and(outakeStalled.negate()))
        .withName("score");
  }

  public Command intake(PieceType type) {
    if (type == PieceType.coral) {
      return intakeCoral();
    } else {
      return intakeAlgae();
    }
  }

  public Command intakeAlgae() {
    return run(() -> requestVoltageControl(12.0)).until(hasAlgae).withName("inake algea");
  }

  public Command intakeCoral() {
    return (run(() -> requestVoltageControl(6.0)).until(manipulatorDetects))
        .andThen(run(() -> requestVoltageControl(3.0)))
        .until(elevatorDetects.negate())
        .andThen(stopRollers())
        .withName("intake coral");
  }

  private boolean motorsStalled() {
    boolean rightMotor = inputs.rightMotorCurrent > 10.0 && inputs.rightMotorVelocity < 5.0;
    boolean leftMotor = inputs.leftMotorCurrent > 10.0 && inputs.leftMotorVelocity < 5.0;

    return rightMotor && leftMotor;
  }
}
