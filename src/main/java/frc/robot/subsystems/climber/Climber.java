package frc.robot.subsystems.climber;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.MechanismConstants.ClimberConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private double climberPOS;

  private static final double prepareClimbPosition = 0.0;
  private static final double climbPosition = 4.6; // may be double this

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
    io.voltageControl(voltage.get());
  }

  public Command voltageCommand(Supplier<Double> voltage) {
    return Commands.run(() -> requestClimberVoltage(voltage));
  }

  public Command prepareClimbPosition() {
    return run(() -> requestPosition(climbPosition));
  }

  private void requestPosition(double position) {
    io.setClimberPosition(position);
  }

  public void setClimberPosition(double pos) {
    io.setClimberPosition(pos);
  }

  public double getClimberPos() {
    return inputs.climberPOS;
  }

  public void stop() {
    io.stop();
  }
}
