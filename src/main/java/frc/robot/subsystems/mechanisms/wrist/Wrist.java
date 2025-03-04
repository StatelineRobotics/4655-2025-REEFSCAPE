package frc.robot.subsystems.mechanisms.wrist;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.mechanisms.MechanismConstants.RollerConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double leftIntakeRPM;
  private double rightIntakeRPM;
  private double wirstPos;
  public Trigger atSetpoint = new Trigger(this::isAtSetpoint);
  public Trigger intakeStalled =
      new Trigger(
              () ->
                  Math.round(inputs.rightAppliedCurrent) >= RollerConstants.currentLimit
                      || Math.round(inputs.leftAppliedCurrent) >= RollerConstants.currentLimit)
          .debounce(.25, DebounceType.kFalling);

  public Trigger detectsNote = new Trigger(() -> inputs.detectsNote);

  public Trigger detectsNoteDebounced = detectsNote.debounce(.25, DebounceType.kRising);

  public Wrist(WristIO io) {
    this.io = io;

    SmartDashboard.putData("Wrist/upperCommand", upperTestCommand());
    SmartDashboard.putData("Wrist/lowerCommand", lowerTestCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    Logger.recordOutput("Wrist/IntakeStalled", intakeStalled);

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

  public void stopWrist() {
    io.stopWrist();
  }

  public void stopIntake() {
    io.stopIntake();
  }

  public void requestWristPOS(double POS) {
    inputs.wristSetpoint = POS;
    io.requestWristPosition(POS);
  }

  public void reqestIntakeVoltage(double voltage) {
    io.requestIntakeVoltage(voltage);
  }

  public Command requestWristPosition(double setPoint) {
    return Commands.runOnce(() -> io.requestWristPosition(setPoint));
  }

  private Command reqestIntakeVoltage(Supplier<Double> voltage) {
    return Commands.run(() -> io.requestIntakeVoltage(voltage.get()));
  }

  public Command wristVoltageControl(Supplier<Double> voltage) {
    return Commands.run(() -> io.requestWristVoltage(voltage.get()));
  }

  public Command intakeVoltageControl(Supplier<Double> voltage) {
    return Commands.run(() -> io.requestIntakeVoltage(voltage.get()));
  }

  public Command upperTestCommand() {
    return this.defer(
        () ->
            Commands.runOnce(
                () -> requestWristPOS(SmartDashboard.getNumber("Wrist/upperSetpoint", 0.0))));
  }

  public Command lowerTestCommand() {
    return this.defer(
        () ->
            Commands.runOnce(
                () -> requestWristPOS(SmartDashboard.getNumber("Wrist/lowerSetpoint", 0.0))));
  }

  public Command stopCommand() {
    return this.runOnce(this::stop);
  }
}
