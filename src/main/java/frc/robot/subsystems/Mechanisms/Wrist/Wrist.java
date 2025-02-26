package frc.robot.subsystems.mechanisms.wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double leftIntakeRPM;
  private double rightIntakeRPM;
  private double wirstPos;

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

  private void stop() {
    io.stop();
  }

  private void requestWristPOS(double POS) {
    wirstPos = POS;
  }

  public Command intakePosition() {
    return requestWristPosition(WristConstants.intakeAngle);
  }

  public Command holdPostion() {
    return requestWristPosition(WristConstants.holdAngle);
  }

  public Command scorePosition() {
    return requestWristPosition(WristConstants.scoreAngle);
  }

  public Command intakeFastSpeed() {
    return requestIntakeSpeed(500000);
  }

  public Command intakeScoreSpeed() {
    return requestIntakeSpeed(50000);
  }

  public Command wristVoltageControl(Supplier<Double> voltage) {
    return Commands.run(() -> io.requestWristVoltage(voltage.get()));
  }

  public Command intakeVoltageControl(Supplier<Double> voltage) {
    return Commands.run(() -> io.requestIntakeVoltage(voltage.get()));
  }

  public Command requestWristPosition(double setPoint) {
    return Commands.runOnce(() -> io.requestWristPosition(setPoint));
  }

  public Command requestIntakeSpeed(double setPoint) {
    return Commands.runOnce(() -> io.requestIntakeVelo(setPoint));
  }

  private void requestIntakeVelo(double RPM) {
    leftIntakeRPM = RPM;
    rightIntakeRPM = RPM;
  }
}
