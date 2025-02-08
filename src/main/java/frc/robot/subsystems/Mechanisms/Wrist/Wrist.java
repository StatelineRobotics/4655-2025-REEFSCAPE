package frc.robot.subsystems.Mechanisms.Wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private WristIOInputsAutoLogged inputs;
  private double leftIntakeRPM;
  private double rightIntakeRPM;
  private double wirstPos;

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    } else {

    }
  }

  private void stop() {
    leftIntakeRPM = 0;
    rightIntakeRPM = 0;
    io.stop();
  }

  public void requestWristPOS(double POS, double RPM) {
    leftIntakeRPM = RPM;
    rightIntakeRPM = RPM;
    wirstPos = POS;
  }
}
