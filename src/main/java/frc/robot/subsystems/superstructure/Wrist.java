package frc.robot.subsystems.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.idle;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.MechanismConstants.WristConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private ArmFeedforward armFeedforward = new ArmFeedforward(.10, .28 - .19, 0);

  @AutoLogOutput public Trigger atSetpoint = new Trigger(this::isAtSetpoint);

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    } else {

    }
  }

  public boolean isAtSetpoint() {
    if (Math.abs(inputs.wristPos - inputs.wristSetpoint) < WristConstants.allowError) {
      return true;
    }
    return false;
  }

  private void stop() {
    io.stop();
  }

  private void requestWristPosition(double POS) {
    inputs.wristSetpoint = POS;
    double feedforward = armFeedforward.calculate(Math.toRadians(-1.0 * inputs.wristPos), 0);
    io.requestWristPosition(POS, feedforward);
  }

  public Command wristVoltageControl(Supplier<Double> voltage) {
    return Commands.run(() -> io.requestWristVoltage(voltage.get()));
  }

  public Command stopCommand() {
    return this.runOnce(this::stop);
  }

  protected Command idleCommand() {
    return idle(this);
  }

  protected Command moveToSetpoint(double position) {
    return run(() -> requestWristPosition(position)).until(atSetpoint);
  }
}
