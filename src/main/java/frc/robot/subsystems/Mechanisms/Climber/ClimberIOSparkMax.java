package frc.robot.subsystems.mechanisms.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.mechanisms.MechanismConstants;

public class ClimberIOSparkMax implements ClimberIO{
    private SparkFlex m_climber;
    private SparkBaseConfig climberConfig;
    private RelativeEncoder climbEncoder;
    private RelativeEncoder funnelEncoder;
    private SparkClosedLoopController climbController;
    private SparkClosedLoopController funnelController;

    public ClimberIOSparkMax(){
        climberConfig.idleMode(IdleMode.kBrake);
        climberConfig.smartCurrentLimit(0);

        m_climber = new SparkFlex(MechanismConstants.climberId, MotorType.kBrushless);
        m_climber.configure(climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        climbEncoder = m_climber.getEncoder();

        climbController = m_climber.getClosedLoopController();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberPOS = climbEncoder.getPosition();
        inputs.funnelPOS = funnelEncoder.getPosition();
    }

    public void setClimberPosition(double pos){
        climbController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void stop(){
        m_climber.stopMotor();
    }
}
