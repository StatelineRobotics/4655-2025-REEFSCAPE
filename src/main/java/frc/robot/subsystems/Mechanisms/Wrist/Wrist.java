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

  DriverStationTriggers.onDisable.onTrue(this::stop);

  public Wrist(WristIO io) {
    this.io = io;
    Smartdashboard.putData("wrist/upperCommand", upperCommand());
    Smartdashboard.putData("wrist/lowerCommand", lowerCommand())
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
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

  public Command upperTestCommand() {
    return this.defer(
      () ->
        Commands.run(
          () -> positionThing(SmartDashboard.get("wrist/upperPosition", 0.0))));
  }

  public Command lowerTestCommand() {
    return this.defer(
      () -> 
        Commands.run(
          () -> postionThing(SmartDashboard.get("wrist/lowerPosition", 0.0))));
  }
}
