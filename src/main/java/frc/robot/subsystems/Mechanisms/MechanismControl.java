package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Mechanisms.Climber.Climber;
import frc.robot.subsystems.Mechanisms.Elevator.Elevator;
import frc.robot.subsystems.Mechanisms.Wrist.Wrist;

public class MechanismControl extends SubsystemBase {

  public enum State {
    algaePickup,
    algaeEject,
    coralPickup,
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

  public MechanismControl(Elevator elevatorSubsystem, Wrist wristSubsystem, Climber climber){
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.climber = climber;
  }

  public void periodic(){

    switch(currentState){
      case home -> {
        if(elevatorSubsystem.okToHome()){
          
        }
      }
    }
    
  }
}
