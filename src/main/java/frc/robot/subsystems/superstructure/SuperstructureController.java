package frc.robot.subsystems.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.FieldConstants.PieceType.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants.PieceType;

public class SuperstructureController {
  private Elevator elevator;
  private Wrist wrist;
  private OutakeRollers outake;
  private Drive drive;

  public Trigger useFullAuto;

  public static enum ScorePositions {
    level1(0.05, -45, 6.0),
    level2(0.05, -45, 6.0);

    public final double elevator;
    public final double wrist;
    public final double outake;

    private ScorePositions(double elevator, double wrist, double outake) {
      this.elevator = elevator;
      this.wrist = wrist;
      this.outake = outake;
    }
  }

  public static enum IntakePositions {
    hopperCoral(1, 1, coral),
    algea(1, 1, algae);

    public final double elevator;
    public final double wrist;
    public final PieceType type;

    private IntakePositions(double elevator, double wrist, PieceType type) {
      this.elevator = elevator;
      this.wrist = wrist;
      this.type = type;
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

  public Command score(ScorePositions position) {
    return idle(outake); // needs to be the outake move shooty thing
  }

  public Command intake(IntakePositions postion) {
    return sequence(wrist.moveToSetpoint(postion.wrist), elevator.moveToSetpoint(postion.elevator))
           .andThen(outake.intake(postion.type));
  }

  public Command prepareToScore(ScorePositions positions) {
    return wrist
        .moveToSetpoint(positions.elevator)
        .alongWith(elevator.moveToSetpoint(positions.wrist));
  }
}
