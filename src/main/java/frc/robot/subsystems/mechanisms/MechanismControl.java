package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Lights;
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

  private final Elevator elevatorSubsystem;
  private final Wrist wristSubsystem;
  private final Climber climber;
  private final Lights lightSubsystem;

  @AutoLogOutput public final Trigger atDualSetPoint;

  private double test = 0.0;
  private boolean hasSetLEDS = false;

  public MechanismControl(
      Elevator elevatorSubsystem, Wrist wristSubsystem, Climber climber, Lights lightSubsystem) {
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
        elevatorSubsystem.requestFunnelPOS(0);
        if (elevatorSubsystem.getFunnelPos() > 10) {
          climber.setClimberPosition(9);
        }
        break;
      }

      case climb -> {
        climber.requestPull();
        break;
      }

      case climberHome -> {
        climber.setClimberPosition(0);
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
            return new InstantCommand(() -> setDesiredState(desiredState))
                .alongWith(getLEDCommand(desiredState));
          });
    } else {
      return Commands.defer(
          () -> {
            return new InstantCommand(() -> setDesiredState(desiredState))
                .alongWith(getLEDCommand(desiredState));
          },
          Set.of(elevatorSubsystem, wristSubsystem, climber));
    }
  }

  public void setDesiredState(State desiredState) {
    currentState = desiredState;
  }

  private Command getLEDCommand(State desiredState) {
    switch (desiredState) {
      case idle, home:
        // Single fade purple annimation
        return lightSubsystem.singleFadeAnimation(new Color(80, 7, 120));

        // When intaking solid red
      case coralPickup:
        return lightSubsystem.singleColorAnimation(new Color(255, 0, 0));

        // When see coral, solid orange
      case coralPickupS3:
        return lightSubsystem.singleColorAnimation(new Color(250, 130, 38));

        // Move elevatlor store (red) when at store blue
      case store, algeaStore:
        return lightSubsystem
            .singleColorAnimation(new Color(255, 0, 0))
            .andThen(Commands.waitUntil(atDualSetPoint))
            .andThen(lightSubsystem.singleColorAnimation(new Color(0, 125, 255)));

        // Starts red when moving, strobes green when at set point
      case levelOne, levelTwo, levelThree, levelFour:
        return lightSubsystem
            .singleColorAnimation(new Color(255, 0, 0))
            .andThen(Commands.waitUntil(atDualSetPoint))
            .andThen(lightSubsystem.singleStrobeAnimation(new Color(157, 232, 46)));

      case algeaPickupL3, algaePickupL2:
        return lightSubsystem
            .singleColorAnimation(new Color(255, 0, 0))
            .andThen(Commands.waitUntil(atDualSetPoint))
            .andThen(lightSubsystem.singleStrobeAnimation(new Color(157, 232, 46)))
            .andThen(Commands.waitUntil(wristSubsystem.intakeStalled))
            .andThen(lightSubsystem.singleStrobeAnimation(new Color(157, 232, 46)));

        // default to yellow solid color
      default:
        return lightSubsystem.singleColorAnimation(new Color(255, 209, 0));
    }
  }
}
