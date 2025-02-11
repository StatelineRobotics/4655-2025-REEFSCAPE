package frc.robot.subsystems.Mechanisms.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final ClimberIO io;

    public Climber(ClimberIO io){
        this.io = io;
    }
}
