package frc.robot.subsystems.mechanisms.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.ClimberConstants;

public class ClimberIOSparkMax implements ClimberIO {
  private SparkFlex m_climber;
  private SparkFlexConfig climberConfig = new SparkFlexConfig();
  private RelativeEncoder climbEncoder;
  private SparkClosedLoopController climbController;
  private SparkClosedLoopController funnelController;
  private ClosedLoopConfig climberClosedLoopConfig = climberConfig.closedLoop;
  private ArmFeedforward armFeedforward =
      new ArmFeedforward(ClimberConstants.ks, ClimberConstants.kg, 0);

  public ClimberIOSparkMax() {
    climberConfig.idleMode(IdleMode.kBrake);
    climberConfig.smartCurrentLimit(0);
    climberConfig.closedLoop.velocityFF(100);

    m_climber = new SparkFlex(MechanismConstants.climberId, MotorType.kBrushless);
    m_climber.configure(
        climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    climbController = m_climber.getClosedLoopController();
    climbEncoder = m_climber.getEncoder();

    climberClosedLoopConfig = climberConfig.closedLoop;
    climberClosedLoopConfig.pid(0.002, 0, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberPOS = climbEncoder.getPosition();
  }

  public void setClimberPosition(double pos) {
    climbController.setReference(pos, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public void requestPull() {
    m_climber.setVoltage(-6);
  }

  public void stop() {
    m_climber.stopMotor();
  }
}
