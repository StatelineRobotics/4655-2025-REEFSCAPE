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
import java.util.function.BooleanSupplier;
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
    algeaStore,
    algeaGround,
    storeDump,
    storeDump2,
    dump,
    reverse
  }

  public State currentState = State.idle;

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

  @AutoLogOutput
  public Trigger getElevatorUp() {
    BooleanSupplier stateCondition =
        () -> {
          return currentState == State.levelOne
              || currentState == State.levelTwo
              || currentState == State.levelThree
              || currentState == State.levelFour;
        };
    return atDualSetPoint.and(stateCondition);
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
          setState(State.coralPickupS2).schedule();
        }
        break;
      }

      case coralPickupS2 -> {
        wristSubsystem.reqestIntakeVoltage(6);
        elevatorSubsystem.reqestBeltVoltage(-12);
        if (wristSubsystem.detectsNoteDebounced.getAsBoolean() == true) {
          setState(State.coralPickupS3).schedule();
        }
        break;
      }

      case coralPickupS3 -> {
        wristSubsystem.reqestIntakeVoltage(4.75);
        if (wristSubsystem.detectsNote.getAsBoolean() == false) {
          wristSubsystem.reqestIntakeVoltage(-1);
          elevatorSubsystem.reqestBeltVoltage(0);
          setState(State.store).schedule();
        }
      }

      case reverse -> {
        elevatorSubsystem.reqestBeltVoltage(12);
      }

        // Idealy this should be the correct height to score algea into processor
      case store -> {
        wristSubsystem.requestWristPOS(WristConstants.storeAngle);
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.storeHeight);
        if (wristSubsystem.intakeStalled.getAsBoolean() == true) {
          setState(State.algeaStore).schedule();
        } else {
          wristSubsystem.stopIntake();
        }
      }

      case algeaStore -> {
        wristSubsystem.requestWristPOS(WristConstants.storeAlgeaAngle);
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.storeAlgeaHeight);
        if (wristSubsystem.intakeStalled.getAsBoolean() == false) {
          setState(State.store).schedule();
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
        wristSubsystem.requestWristPOS(WristConstants.algeaIntakeAngle);
        wristSubsystem.reqestIntakeVoltage(-6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.reqestIntakeVoltage(-6);
        }
      }

      case algeaPickupL3 -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.algeaL3);
        wristSubsystem.requestWristPOS(WristConstants.algeaIntakeAngle);
        wristSubsystem.reqestIntakeVoltage(-6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.reqestIntakeVoltage(-6);
        }
      }

      case climberPrep -> {
        elevatorSubsystem.requestFunnelPOS(105);
        if (elevatorSubsystem.getFunnelPos() > 90) {
          climber.setClimberPosition(0);
        }
        break;
      }

      case climb -> {
        driveSubsystem.setWheelsStraightAndCoast();
        climber.requestPull();
        if (climber.getClimberPos() >= 11.0) {
          climber.stop();
          setState(State.idle);
        }
        break;
      }

      case climberHome -> {
        climber.setClimberPosition(0);
        break;
      }

      case storeDump -> {
        climber.setClimberPosition(0);
        break;
      }

      case algeaGround -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.algeaGround);
        wristSubsystem.requestWristPOS(WristConstants.algeaGround);
        wristSubsystem.reqestIntakeVoltage(-6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          setState(State.algeaStore);
        }
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
            return Commands.runOnce(() -> setDesiredState(desiredState));
          });
    } else {
      return Commands.defer(
          () -> {
            return Commands.runOnce(() -> setDesiredState(desiredState));
          },
          Set.of(elevatorSubsystem, wristSubsystem, climber));
    }
  }

  public void setDesiredState(State desiredState) {
    currentState = desiredState;
    periodic();
    getLEDCommand(desiredState).schedule();
  }

  private Command getLEDCommand(State desiredState) {
    Command command;
    switch (desiredState) {
      case idle, home:
        // Solid purple
        command = lightSubsystem.solidAnimation(new Color(80, 7, 120), "Solid Purple");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // When intaking solid red
      case coralPickup:
        command = lightSubsystem.solidAnimation(new Color(255, 0, 0), "Solid Red");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // When see coral, strobe red
      case coralPickupS3:
        command = lightSubsystem.strobeAnimation(new Color(255, 0, 0), "Strobe Red");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // Move elevatlor store (red) when at store blue
      case store, algeaStore:
        command =
            lightSubsystem
                .solidAnimation(new Color(255, 0, 0), atDualSetPoint, "Solid Red")
                .andThen(lightSubsystem.solidAnimation(new Color(0, 0, 255), "Solid Blue"));
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // Starts red when moving, soid green when at set point
      case levelOne, levelTwo, levelThree, levelFour:
        command =
            lightSubsystem
                .solidAnimation(new Color(255, 0, 0), atDualSetPoint, "Solid Red")
                .andThen(lightSubsystem.solidAnimation(new Color(0, 255, 0), "Solid Green"));
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

      case algeaPickupL3, algaePickupL2:
        command =
            lightSubsystem
                .solidAnimation(new Color(255, 0, 0), atDualSetPoint, "Solid Red")
                .andThen(
                    lightSubsystem.solidAnimation(
                        new Color(0, 0, 255), wristSubsystem.intakeStalled, "Solid Green"))
                .andThen(lightSubsystem.strobeAnimation(new Color(0, 255, 0), "Strobe Green"));
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;

        // default to yellow solid color
      default:
        command = lightSubsystem.solidAnimation(new Color(255, 209, 0), "Solid YELLOW");
        Logger.recordOutput("MechanismControl/latestLED", command.getName());
        return command;
    }
  }
}
