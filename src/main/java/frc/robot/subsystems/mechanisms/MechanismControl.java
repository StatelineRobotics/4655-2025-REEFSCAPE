package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.MechanismConstants.ElevatorConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;
import frc.robot.subsystems.mechanisms.climber.Climber;
import frc.robot.subsystems.mechanisms.elevator.Elevator;
import frc.robot.subsystems.mechanisms.wrist.Wrist;
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
    coralPickupS3
  }

  private State currentState = State.idle;

  private final Elevator elevatorSubsystem;
  private final Wrist wristSubsystem;
  private final Climber climber;
  private double test = 0.0;

  public MechanismControl(Elevator elevatorSubsystem, Wrist wristSubsystem, Climber climber) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.climber = climber;
    SmartDashboard.getNumber("test", test);
  }

  public void periodic() {
    Logger.recordOutput("MechanismControl/currentState", currentState);

    switch (currentState) {
      case idle -> {
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
        wristSubsystem.reqestIntakeVoltage(3);
        elevatorSubsystem.reqestBeltVoltage(6);
        if (wristSubsystem.detectsNote.getAsBoolean()) {
          setDesiredState(State.coralPickupS3);
        }
        break;
      }

      case coralPickupS3 -> {
        wristSubsystem.reqestIntakeVoltage(1);
        if (!wristSubsystem.detectsNote.getAsBoolean()) {
          wristSubsystem.stopIntake();
          elevatorSubsystem.reqestBeltVoltage(0);
          setDesiredState(State.store);
        }
      }

      //Idealy this should be the correct height to score algea into processor
      case store -> {
        wristSubsystem.requestWristPOS(WristConstants.storeAngle);
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.storeHeight);
        if (!wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.stopIntake();
        }
      }

        // Also used as score
      case eject -> {
        wristSubsystem.reqestIntakeVoltage(12);
        break;
      }

      case levelOne -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.levelOne);
        wristSubsystem.requestWristPOS(WristConstants.coralScoreAngle);
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
        wristSubsystem.reqestIntakeVoltage(6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.reqestIntakeVoltage(1);
        }
      }

      case algeaPickupL3 -> {
        elevatorSubsystem.requestElevatorPosition(ElevatorConstants.algeaL3);
        wristSubsystem.requestWristPosition(WristConstants.algeaIntakeAngle);
        wristSubsystem.reqestIntakeVoltage(6);
        if (wristSubsystem.intakeStalled.getAsBoolean()) {
          wristSubsystem.reqestIntakeVoltage(1);
        }
      }

      case climberPrep -> {
        elevatorSubsystem.requestFunnelPOS(0);

        break;
      }

      case climberHome -> {
        climber.setClimberPosition(0);
        break;
      }
    }
  }

  //Just a shorthand for setting state with commands to avoid needing more repetition in RobotContainer
  public Command setState(State desiredState) {
    return Commands.deferredProxy(
        () -> {
          return new InstantCommand(() -> setDesiredState(desiredState));
        });
  }

  public void setDesiredState(State desiredState) {

    currentState = desiredState;
  }
}
