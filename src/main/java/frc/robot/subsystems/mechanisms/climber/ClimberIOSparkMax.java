package frc.robot.subsystems.mechanisms.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.mechanisms.MechanismConstants;

public class ClimberIOSparkMax implements ClimberIO {
  private SparkFlex m_climber;
  private SparkFlexConfig climberConfig = new SparkFlexConfig();
  private RelativeEncoder climbEncoder;
  private AbsoluteEncoder climberEncoder;
  private SparkClosedLoopController climbController;
  private ClosedLoopConfig climberClosedLoopConfig = climberConfig.closedLoop;

  // private ArmFeedforward armFeedforward =
  //     new ArmFeedforward(ClimberConstants.ks, ClimberConstants.kg, 0);

  public ClimberIOSparkMax() {
    climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(0).inverted(false);

    climberConfig.closedLoop.pid(0.5, 0, 0);

    m_climber = new SparkFlex(MechanismConstants.climberId, MotorType.kBrushless);
    m_climber.configure(
        climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    climbEncoder = m_climber.getEncoder();
    climberEncoder = m_climber.getAbsoluteEncoder();

    climbController = m_climber.getClosedLoopController();
    climbEncoder = m_climber.getEncoder();

    climberClosedLoopConfig =
        climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    climberClosedLoopConfig.pid(0.002, 0, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberPOS = climbEncoder.getPosition();
    inputs.climberCurrent = m_climber.getBusVoltage() * m_climber.getAppliedOutput();
    // inputs.funnelPOS = funnelEncoder.getPosition();
  }

  public void setClimberPosition(double pos) {
    climbController.setReference(pos, SparkBase.ControlType.kPosition);
  }

  public void requestPull() {
    m_climber.setVoltage(12);
  }

  public void stop() {
    m_climber.stopMotor();
  }
}
