package frc.robot.subsystems.mechanisms.climber;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.mechanisms.MechanismConstants.ClimberConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private double climberPOS;
  private double funnelPOS;
  public Trigger climberStalled =
      new Trigger(() -> Math.round(inputs.climberCurrent) >= ClimberConstants.climberCurrentLimit)
          .debounce(.25, DebounceType.kFalling);

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void requestClimberVoltage(Supplier<Double> voltage) {
    io.voltageControl(12 * voltage.get());
  }

  public void requestPull() {
    io.requestPull();
  }

  public void reqestPosition(double position) {
    io.setClimberPosition(position / 2.0);
  }

  public void setClimberPosition(double pos) {
    io.setClimberPosition(pos / 2.0);
  }

  public double getClimberPos() {
    return inputs.climberPOS;
  }

  public void stop() {
    io.stop();
  }
}
