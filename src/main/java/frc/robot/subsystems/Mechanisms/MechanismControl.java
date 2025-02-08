package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public MechanismControl(Elevator elevatorSubsystem, Wrist wristSubsystem){
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
  }

  public void periodic(){
    
  }
}
