package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.Drive;
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
    barge,
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
  private boolean hasSetElevatorPosition = false;

  public Trigger elevatorUp =
      new Trigger(
              () -> {
                return currentState == State.levelOne
                    || currentState == State.levelTwo
                    || currentState == State.levelThree
                    || currentState == State.levelFour;
              })
          .debounce(.25);

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
    this.atDualSetPoint =
        new Trigger(wristSubsystem.atSetpoint.and(elevatorSubsystem.elevatorAtSetpoint));
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
        wristSubsystem.requestWristPOS(0);
        wristSubsystem.stopIntake();

        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getCoralStoreCommand().schedule();
        }

        break;
      }

      case coralPickup -> {
        wristSubsystem.requestWristPOS(WristConstants.intakeCoralAngle);
        elevatorSubsystem.requestFunnelPOS(0.0);

        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getIntakeCommand().schedule();
        }

        if (elevatorSubsystem.isAtSetpoint()
            && wristSubsystem.isAtSetpoint()
            && elevatorSubsystem.getFunnelPos() < 5.0) {
          setState(State.coralPickupS2).schedule();
        }
        break;
      }

      case coralPickupS2 -> {
        wristSubsystem.reqestIntakeVoltage(3);
        elevatorSubsystem.reqestBeltVoltage(-8);
        elevatorSubsystem.requestFunnelPOS(0.0);

        if (!hasSetElevatorPosition) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getIntakeCommand().schedule();
        }

        if (wristSubsystem.detectsBoth.getAsBoolean()) {
          setState(State.coralPickupS3).schedule();
        }
        break;
      }

      case coralPickupS3 -> {
        wristSubsystem.reqestIntakeVoltage(2.0);

        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getIntakeCommand().schedule();
        }

        if (!wristSubsystem.detectsNote.getAsBoolean()) {
          wristSubsystem.stopIntake();
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

        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getCoralStoreCommand().schedule();
        }

        if (wristSubsystem.intakeStalled.getAsBoolean() == true
            && wristSubsystem.detectsForward.getAsBoolean() == false) {
          setState(State.algeaStore).schedule();
        } else {
          wristSubsystem.stopIntake();
        }
      }

      case algeaStore -> {
        wristSubsystem.requestWristPOS(WristConstants.storeAlgeaAngle);
        wristSubsystem.reqestIntakeVoltage(-1);

        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getAlgaeStoreCommand().schedule();
        }

        if (wristSubsystem.intakeStalled.getAsBoolean() == false
            || wristSubsystem.detectsForward.getAsBoolean() == true) {
          setState(State.store).schedule();
        }
      }

        // Also used as score
      case eject -> {
        wristSubsystem.reqestIntakeVoltage(12);
        break;
      }

      case levelOne -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getL1Command().schedule();
        }

        wristSubsystem.requestWristPOS(WristConstants.l1angle);
        break;
      }

      case levelTwo -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getL2Command().schedule();
        }

        wristSubsystem.requestWristPOS(WristConstants.coralScoreAngle);
        break;
      }

      case levelThree -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getL3Command().schedule();
        }

        wristSubsystem.requestWristPOS(WristConstants.coralScoreAngle);
        break;
      }

      case levelFour -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getL4Command().schedule();
        }

        wristSubsystem.requestWristPOS(WristConstants.L4coralScoreAngle);
        break;
      }

      case barge -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getBargeCommandI().schedule();
        }
        wristSubsystem.requestWristPOS(WristConstants.bargeangle);
        wristSubsystem.reqestIntakeVoltage(-1);
      }

      case algaePickupL2 -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getAlgaeL2Command().schedule();
        }

        wristSubsystem.requestWristPOS(WristConstants.algeaIntakeAngle);
        wristSubsystem.reqestIntakeVoltage(-6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.reqestIntakeVoltage(-6);
        }
      }

      case algeaPickupL3 -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getAlgaeL3Command().schedule();
        }

        wristSubsystem.requestWristPOS(WristConstants.algeaIntakeAngle);
        wristSubsystem.reqestIntakeVoltage(-6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.reqestIntakeVoltage(-6);
        }
      }

      case climberPrep -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getIntakeCommand().schedule();
        }
        wristSubsystem.requestWristPOS(-80);
        elevatorSubsystem.requestFunnelPOS(100);
        elevatorSubsystem.reqestBeltVoltage(0);
        if (elevatorSubsystem.getFunnelPos() > 90) {
          climber.setClimberPosition(0);
        }
        break;
      }

      case climb -> {
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getIntakeCommand().schedule();
        }
        wristSubsystem.requestWristPOS(-80);
        driveSubsystem.setWheelsStraightAndCoast();
        climber.requestPull();
        if (climber.getClimberPos() >= 9.25) {
          climber.stop();
          setState(State.idle).schedule();
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
        if (!hasSetElevatorPosition && !wristSubsystem.detectsNote.getAsBoolean()) {
          hasSetElevatorPosition = true;
          elevatorSubsystem.getAlgaeIntakeCommand().schedule();
        }

        wristSubsystem.requestWristPOS(WristConstants.algeaGround);
        wristSubsystem.reqestIntakeVoltage(-6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          setState(State.algeaStore);
        }
        break;
      }
    }
  }

  public Command setIdleState() {
    return Commands.runOnce(() -> setDesiredState(State.idle));
  }

  public Command setNewState(Supplier<State> desiredState) {
  return Commands.runOnce(() -> setDesiredState(desiredState.get()),
                          Set.of(elevatorSubsystem, wristSubsystem, climber))
                          .withName("state: " + desiredState.get());
}

public Command setNewScoreState(Supplier<State> desiredState) {
  return (Commands.waitUntil(!driveSubsystem.firstStageAuto).withName("Wait For AutoAlign"))
          .andThen(setNewState(desiredState));

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
            return Commands.waitUntil(
                    () ->
                        desiredState == State.store
                            || desiredState == State.algeaStore
                            || desiredState == State.coralPickup
                            || !driveSubsystem.firstStageAuto)
                .andThen(Commands.runOnce(() -> setDesiredState(desiredState)));
          },
          Set.of(elevatorSubsystem, wristSubsystem, climber));
    }
  }

  public void setDesiredState(State desiredState) {
    elevatorSubsystem.reqestBeltVoltage(0);
    hasSetElevatorPosition = false;
    currentState = desiredState;
    periodic();
    // getLEDCommand(desiredState).schedule();
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
        command = lightSubsystem.strobeAnimation(new Color(0, 255, 0), "strobeGreen");
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
