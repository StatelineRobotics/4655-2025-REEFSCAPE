package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.elevator.Elevator;
import frc.robot.subsystems.mechanisms.wrist.Wrist;

public class SuperstructureController {
  private Elevator elevator;
  private Wrist wrist;
  private Drive drive;

  public Trigger useFullAuto;

  public SuperstructureController(Drive drive, Elevator elevator, Wrist wrist) {
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    SmartDashboard.putBoolean("useFullAuto", true);
    useFullAuto = new Trigger(() -> SmartDashboard.getBoolean("useFullAuto", false));
  }

  private Command autoCoralScoreCommand(Command elevatorPosition, Command wristPosition) {
    return Commands.waitUntil(drive.autoElevator)
                   .andThen(elevatorPosition.alongWith(wristPosition))
                   .andThen(Commands.waitUntil(drive.readyAutoScore))
                   //.andThen(someThing someThing score command)
                   .andThen(wrist.moveCoralStorePosition()).withTimeout(Seconds.of(.5))
                   .andThen(DriveCommands.straightBack(drive).until(drive.safeElevatorDown))
                   .andThen(wrist.moveCoralStorePosition().alongWith(elevator.getCoralStoreCommand()));
  }

  private Command manualCoralScoreCommand(Command elevatorPosition, Command wristPosition) {
    return elevatorPosition.alongWith(wristPosition);
  }

  private Command coralScoreCommand(Command elevatorPosition, Command wristPosition) {
    return new ConditionalCommand(autoCoralScoreCommand(elevatorPosition, wristPosition), manualCoralScoreCommand(elevatorPosition, wristPosition), useFullAuto);
  }

  public Command l2ScoreCommand() {
    return coralScoreCommand(elevator.getL2Command(), wrist.moveMidcoralPosition()).withName("L2 score command");
  }

  public Command l3ScoreCommand() {
    return coralScoreCommand(elevator.getL3Command(), wrist.moveMidcoralPosition()).withName("L3 score command");
  }

  public Command l4ScoreCommand() {
    return coralScoreCommand(elevator.getL4Command(), wrist.moveL4coralPosition()).withName("L4 score command");
  }

}
