package frc.robot.subsystems.Mechanisms.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.subsystems.Mechanisms.MechanismConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private SparkMax m_leftElevator;
  private SparkMax m_rightElevator;
  private SparkClosedLoopController leftElevatorController;
  private SparkClosedLoopController rightElevatorController;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private SparkLimitSwitch limitSwitch;
  private boolean zeroed;
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
    m_leftElevator.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightElevator.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = m_leftElevator.getEncoder();
    rightEncoder = m_rightElevator.getEncoder();

    limitSwitch = m_leftElevator.getReverseLimitSwitch();

    leftElevatorController = m_leftElevator.getClosedLoopController();
    rightElevatorController = m_rightElevator.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftElevatorPosition = leftEncoder.getPosition();
    inputs.rightElevatorPosition = rightEncoder.getPosition();

    if (limitSwitch.isPressed()) {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      zeroed = true;
    }
  }

  public void setElevatorPosition(Double climberPosition) {
    if(zeroed){
    leftElevatorController.setReference(
        climberPosition, SparkBase.ControlType.kMAXMotionPositionControl);
    // rightElevatorController.setReference(
    //     climberPosition, SparkBase.ControlType.kMAXMotionPositionControl);
    } else {
      leftElevatorController.setReference(0, SparkBase.ControlType.kVoltage);
      // rightElevatorController.setReference(-100, SparkBase.ControlType.kMAXMotionPositionControl);
    }
  }

  public void stop() {
    m_leftElevator.stopMotor();
    m_rightElevator.stopMotor();
  }
}
