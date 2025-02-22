package frc.robot.subsystems.mechanisms.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.subsystems.mechanisms.MechanismConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private SparkMax m_leftElevator;
  private SparkMax m_rightElevator;
  private SparkFlex m_funnel;
  private SparkMax m_belt;
  private SparkClosedLoopController leftElevatorController;
  private SparkClosedLoopController rightElevatorController;
  private SparkClosedLoopController funnelController;
  private SparkClosedLoopController beltController;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder funnelEncoder;
  private SparkLimitSwitch limitSwitch;
  private static boolean zeroed;
  private SparkBaseConfig mLeftConfig;
  private SparkBaseConfig mRightConfig;


  public ElevatorIOSparkMax() {
    mLeftConfig.idleMode(IdleMode.kBrake);
    mLeftConfig.inverted(false);
    mLeftConfig.smartCurrentLimit(0);
    mRightConfig = mLeftConfig;
    mRightConfig.follow(MechanismConstants.leftElevatorId);

    
    m_leftElevator = new SparkMax(MechanismConstants.leftElevatorId, MotorType.kBrushless);
    m_rightElevator = new SparkMax(MechanismConstants.rightElevatorId, MotorType.kBrushless);
    m_funnel = new SparkFlex(MechanismConstants.funnelId, MotorType.kBrushless);
    m_belt = new SparkMax(MechanismConstants.beltId, MotorType.kBrushless);


    m_leftElevator.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightElevator.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = m_leftElevator.getEncoder();
    rightEncoder = m_rightElevator.getEncoder();
    funnelEncoder = m_funnel.getEncoder();

    limitSwitch = m_leftElevator.getReverseLimitSwitch();

    leftElevatorController = m_leftElevator.getClosedLoopController();
    rightElevatorController = m_rightElevator.getClosedLoopController();
    funnelController = m_funnel.getClosedLoopController();
    beltController = m_belt.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorPos = leftEncoder.getPosition();
    inputs.zeroed = zeroed;
    if (limitSwitch.isPressed()) {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      zeroed = true;
    }
  }


  public void requestElevatorPosition(Double climberPosition) {
    if(zeroed){
      zeroed = false;
    leftElevatorController.setReference(
        climberPosition, SparkBase.ControlType.kMAXMotionPositionControl);
    // rightElevatorController.setReference(
    //     climberPosition, SparkBase.ControlType.kMAXMotionPositionControl);
    } else {
      leftElevatorController.setReference(-10, SparkBase.ControlType.kVelocity);
      // rightElevatorController.setReference(-100, SparkBase.ControlType.kMAXMotionPositionControl);
    }
  }

  public void requestFunnelPOS(double POS){
    funnelController.setReference(POS, ControlType.kPosition);
  }

  public void requestBeltRPM(double RPM){
    funnelController.setReference(RPM, ControlType.kVelocity);
  }

  public void stop() {
    m_leftElevator.stopMotor();
    m_rightElevator.stopMotor();
  }
}
