package frc.robot.subsystems.mechanisms.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase{
    private final ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private double climberPOS;
    private double funnelPOS;

    public Climber(ClimberIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }

    public void setFunnelPosition(double climberPOS){
        inputs.funnelPOS = funnelPOS;
    }

    public void setClimberPosition(double pos){
       inputs.climberPOS = climberPOS;
    }
}
