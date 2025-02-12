package frc.robot.subsystems.mechanisms.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double ElevatorPosition = 0.0;

  public Elevator(ElevatorIO io) {
    //  System.out.println("[Init] Creating Elevator");
    this.io = io;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TestArmAngle", 22.5);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    } else {

    }
  }

  public void setElevatorPosition(double ElevatorPosition) {
    this.ElevatorPosition = ElevatorPosition;
  }

  private void stop() {
    ElevatorPosition = 0;
    io.stop();
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }

  public boolean okToHome(){
    return inputs.zeroed;
  }

  @AutoLogOutput(key = "Elevator/OKToReach")
  public boolean ElevatorOkToReach() {
    return (inputs.rightElevatorPosition < 0 && inputs.leftElevatorPosition < 0);
  }
}
