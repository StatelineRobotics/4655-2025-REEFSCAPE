package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.MechanismConstants.ElevatorConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;
import frc.robot.subsystems.mechanisms.climber.Climber;
import frc.robot.subsystems.mechanisms.elevator.Elevator;
import frc.robot.subsystems.mechanisms.wrist.Wrist;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class MechanismControl extends SubsystemBase {

  public enum State {
    idle,
    algaePickupL2,
    algeaPickupL3,
    coralPickup,
    eject,
    levelOne,
    levelTwo,
    levelThree,
    levelFour,
    home,
    climberHome,
    climberPrep,
    climb,
    store,
    coralPickupS2,
    coralPickupS3,
    algeaStore
  }

  private State currentState = State.idle;

  private final Drive driveSubsystem;
  private final Elevator elevatorSubsystem;
  private final Wrist wristSubsystem;
  private final Climber climber;
  private final Lights lightSubsystem;

  @AutoLogOutput public final Trigger atDualSetPoint;

  private double test = 0.0;
  private boolean hasSetLEDS = false;

  public MechanismControl(
      Drive driveSubsystem,
      Elevator elevatorSubsystem,
      Wrist wristSubsystem,
      Climber climber,
      Lights lightSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.climber = climber;
    this.lightSubsystem = lightSubsystem;
    this.atDualSetPoint = new Trigger(wristSubsystem.atSetpoint.and(elevatorSubsystem.atSetpoint));
    SmartDashboard.getNumber("test", test);
  }

  public void periodic() {
    Logger.recordOutput("MechanismControl/currentState", currentState);

    switch (currentState) {
      case idle -> {
        elevatorSubsystem.reqestBeltVoltage(0);
        break;
      }

      case home -> {
        climber.setClimberPosition(test);
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.intakeHeight);
        wristSubsystem.requestWristPOS(0);
        wristSubsystem.stopIntake();
        break;
      }

      case coralPickup -> {
        wristSubsystem.requestWristPOS(WristConstants.intakeCoralAngle);
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.intakeHeight);
        if (elevatorSubsystem.isAtSetpoint() && wristSubsystem.isAtSetpoint()) {
          setDesiredState(State.coralPickupS2);
        }
        break;
      }

      case coralPickupS2 -> {
        wristSubsystem.reqestIntakeVoltage(6);
        elevatorSubsystem.reqestBeltVoltage(-12);
        if (wristSubsystem.detectsNoteDebounced.getAsBoolean() == true) {
          setDesiredState(State.coralPickupS3);
        }
        break;
      }

      case coralPickupS3 -> {
        wristSubsystem.reqestIntakeVoltage(3);
        if (wristSubsystem.detectsNote.getAsBoolean() == false) {
          wristSubsystem.stopIntake();
          elevatorSubsystem.reqestBeltVoltage(0);
          setDesiredState(State.store);
        }
      }

        // Idealy this should be the correct height to score algea into processor
      case store -> {
        wristSubsystem.requestWristPOS(WristConstants.storeAngle);
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.storeHeight);
        if (wristSubsystem.intakeStalled.getAsBoolean() == true) {
          setDesiredState(State.algeaStore);
        } else {
          wristSubsystem.stopIntake();
        }
      }

      case algeaStore -> {
        wristSubsystem.requestWristPOS(WristConstants.storeAlgeaAngle);
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.storeAlgeaHeight);
        if (wristSubsystem.intakeStalled.getAsBoolean() == false) {
          setDesiredState(State.store);
        }
      }

        // Also used as score
      case eject -> {
        wristSubsystem.reqestIntakeVoltage(12);
        break;
      }

      case levelOne -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.levelOne);
        wristSubsystem.requestWristPOS(WristConstants.storeAngle);
        break;
      }

      case levelTwo -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.levelTwo);
        wristSubsystem.requestWristPOS(WristConstants.coralScoreAngle);
        break;
      }

      case levelThree -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.levelThree);
        wristSubsystem.requestWristPOS(WristConstants.coralScoreAngle);
        break;
      }

      case levelFour -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.levelFour);
        wristSubsystem.requestWristPOS(WristConstants.coralScoreAngle);
        break;
      }

      case algaePickupL2 -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.algeaL2);
        wristSubsystem.requestWristPosition(WristConstants.algeaIntakeAngle);
        wristSubsystem.reqestIntakeVoltage(-6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.reqestIntakeVoltage(-6);
        }
      }

      case algeaPickupL3 -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.algeaL3);
        wristSubsystem.requestWristPosition(WristConstants.algeaIntakeAngle);
        wristSubsystem.reqestIntakeVoltage(-6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.reqestIntakeVoltage(-6);
        }
      }

      case climberPrep -> {
        elevatorSubsystem.requestFunnelPOS(124);
        if (elevatorSubsystem.getFunnelPos() > 100) {
          climber.setClimberPosition(-22.3);
        }
        break;
      }

      case climb -> {
        driveSubsystem.coast();
        climber.requestPull();
        if (climber.climberStalled.getAsBoolean()) {
          climber.stop();
        }
        break;
      }

      case climberHome -> {
        climber.setClimberPosition(2);
        break;
      }
    }
  }

  // Just a shorthand for setting state with commands to avoid needing more repetition in
  // RobotContainer
  public Command setState(State desiredState) {
    if (desiredState == State.idle) {
      return Commands.deferredProxy(
          () -> {
            return Commands.runOnce(() -> setDesiredState(desiredState))
                .alongWith(getLEDCommand(desiredState));
            // .alongWith(
            //     Commands.runOnce(
            //         () -> lightSubsystem.singleStrobeAnimation(new Color(255, 0, 0))));
          });
    } else {
      return Commands.defer(
          () -> {
            return Commands.runOnce(() -> setDesiredState(desiredState))
                // .alongWith(
                //     Commands.runOnce(
                //         () -> lightSubsystem.singleStrobeAnimation(new Color(255, 0, 0))));
                .alongWith(getLEDCommand(desiredState));
          },
          Set.of(elevatorSubsystem, wristSubsystem, climber, lightSubsystem));
    }
  }

  public void setDesiredState(State desiredState) {
    currentState = desiredState;
  }

  private Command getLEDCommand(State desiredState) {
    Command command;
    switch (desiredState) {
      case idle, home:
        // Solid purple
        command =
            Commands.runOnce(() -> lightSubsystem.singleColorAnimation(new Color(0, 7, 120)))
                .withName("Solid PURPLE");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // When intaking solid red
      case coralPickup:
        command =
            Commands.runOnce(() -> lightSubsystem.singleColorAnimation(new Color(255, 0, 0)))
                .withName("Solid RED");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // When see coral, solid orange
      case coralPickupS3:
        command =
            Commands.runOnce(() -> lightSubsystem.singleColorAnimation(new Color(250, 130, 38)))
                .withName("Solid ORANGE");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // Move elevatlor store (red) when at store blue
      case store, algeaStore:
        command =
            (Commands.runOnce(() -> lightSubsystem.singleColorAnimation(new Color(255, 0, 0)))
                    .andThen(Commands.waitUntil(atDualSetPoint))
                    .andThen(
                        Commands.runOnce(
                            () -> lightSubsystem.singleColorAnimation(new Color(0, 125, 255)))))
                .withName("Solid RED then Solid BLUE");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // Starts red when moving, strobes green when at set point
      case levelOne, levelTwo, levelThree, levelFour:
        command =
            (Commands.runOnce(() -> lightSubsystem.singleColorAnimation(new Color(255, 0, 0)))
                    .andThen(Commands.waitUntil(atDualSetPoint))
                    .andThen(
                        Commands.runOnce(
                            () -> lightSubsystem.singleStrobeAnimation(new Color(157, 232, 46)))))
                .withName("Solid RED then Strobe GREEN");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

      case algeaPickupL3, algaePickupL2:
        command =
            (Commands.runOnce(() -> lightSubsystem.singleColorAnimation(new Color(255, 0, 0)))
                    .andThen(Commands.waitUntil(atDualSetPoint))
                    .andThen(
                        Commands.runOnce(
                            () -> lightSubsystem.singleStrobeAnimation(new Color(157, 232, 46))))
                    .andThen(Commands.waitUntil(wristSubsystem.intakeStalled))
                    .andThen(
                        Commands.runOnce(
                            () -> lightSubsystem.singleStrobeAnimation(new Color(157, 232, 46)))))
                .withName("Solid RED then Solid GREEN then Strobe GREEN");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // default to yellow solid color
      default:
        command =
            Commands.runOnce(() -> lightSubsystem.singleColorAnimation(new Color(255, 209, 0)))
                .withName("Solid YELLOW");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;
    }
  }
}
