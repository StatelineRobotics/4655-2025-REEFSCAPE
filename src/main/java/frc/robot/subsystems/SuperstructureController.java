package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.elevator.Elevator;
import frc.robot.subsystems.mechanisms.outakeRollers.OutakeRollers;
import frc.robot.subsystems.mechanisms.wrist.Wrist;

public class SuperstructureController {
  private Elevator elevator;
  private Wrist wrist;
  private OutakeRollers outake;
  private Drive drive;

  public Trigger useFullAuto;

  public enum SuperstructurePositions {
    level1(0.05, -45, 6.0),
    level2(0.05, -45, 6.0);

    public final double elevator;
    public final double wrist;
    public final double outake;

    private SuperstructurePositions(double elevator, double wrist, double outake) {
      this.elevator = elevator;
      this.wrist = wrist;
      this.outake = outake;
    }
  }

  public SuperstructureController(
      Drive drive, Elevator elevator, Wrist wrist, OutakeRollers outake) {
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.outake = outake;
    SmartDashboard.putBoolean("useFullAuto", true);
    useFullAuto = new Trigger(() -> SmartDashboard.getBoolean("useFullAuto", false));
  }

  public Command score(SuperstructurePositions position) {
    return idle(outake); // needs to be the outake move shooty thing
  }

  public Command moveToPosition(SuperstructurePositions positions) {
    return wrist.moveToSetpoint(positions).alongWith(elevator.moveToSetpoint(positions));
  }
}
