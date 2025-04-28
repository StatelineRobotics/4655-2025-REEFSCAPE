package frc.robot.subsystems.mechanisms.outakeRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.outakeRollers.OutakeRollersIO.OutakeRollersIOInputs;

import org.littletonrobotics.junction.Logger;

public class OutakeRollers extends SubsystemBase {

  private final OutakeRollersIO io;
  private final OutakeRollersIOInputsAutoLogged inputs = new OutakeRollersIOInputsAutoLogged();

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
}
