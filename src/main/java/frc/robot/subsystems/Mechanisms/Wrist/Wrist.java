package frc.robot.subsystems.mechanisms.wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;

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
    io.stop();
  }

  public void requestWristPOS(double POS) {
    wirstPos = POS;
  }

    public void requestIntake(double RPM){
    leftIntakeRPM = RPM;
    rightIntakeRPM = RPM;
  }
}
