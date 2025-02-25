package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;
import frc.robot.subsystems.mechanisms.climber.Climber;
import frc.robot.subsystems.mechanisms.elevator.Elevator;
import frc.robot.subsystems.mechanisms.wrist.Wrist;

public class MechanismControl extends SubsystemBase {

  public enum State {
    algaePickup,
    coralPickup,
    eject,
    levelOne,
    levelTwo,
    levelThree,
    levelFour,
    home,
    climberHome,
    climberPrep,
    climb
  }

  private State currentState = State.home;

  private final Elevator elevatorSubsystem;
  private final Wrist wristSubsystem;
  private final Climber climber;
  private double test = 0.0;

  

  public MechanismControl(Elevator elevatorSubsystem, Wrist wristSubsystem, Climber climber){
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.climber = climber;
    SmartDashboard.getNumber("test", test);
  }

  public void periodic(){

    switch(currentState){
      case home -> {
        if(elevatorSubsystem.isHomed()){
          elevatorSubsystem.requestElevatorPosition(0);
          climber.setClimberPosition(test);
        }
        elevatorSubsystem.requestElevatorPosition(-10);
        break;
      }
    
      
    case coralPickup -> {
      wristSubsystem.requestIntake(test);
      break;
    }

    case eject -> {
      wristSubsystem.requestIntake(-0);
      break;
    }

    case levelOne ->{
      elevatorSubsystem.requestElevatorPosition(elevatorSubsystem.get1stStageHeight());
      wristSubsystem.requestWristPOS(WristConstants.coralAngle);
      break;
    }

    case levelTwo ->{
      elevatorSubsystem.requestElevatorPosition(elevatorSubsystem.get2ndStageHeight());
      wristSubsystem.requestWristPOS(WristConstants.coralAngle);
      break;
    }

    case levelThree ->{
      elevatorSubsystem.requestElevatorPosition(0);
      wristSubsystem.requestWristPOS(WristConstants.coralAngle);
      break;
    }

    case climberPrep ->{
      elevatorSubsystem.requestFunnelPOS(0);

      break;
    }

    case climberHome ->{
      climber.setClimberPosition(0);
      break;
    }
  }

  
  }
  public void setDesiredState(State desiredState) {

            currentState = desiredState;
        
      }
}
