package frc.robot.subsystems.superstructure;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfigAccessor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.MechanismConstants;
import frc.robot.subsystems.MechanismConstants.ElevatorConstants;
import frc.robot.subsystems.MechanismConstants.FunnelConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
  protected SparkMax m_leftElevator;
  protected SparkMax m_rightElevator;
  protected SparkMax m_funnel;
  protected SparkMax m_belt;
  private SparkClosedLoopController leftElevatorController;
  private SparkClosedLoopController funnelController;
  private SparkClosedLoopController beltController;
  private RelativeEncoder leftEncoder;
  private AbsoluteEncoder funnelEncoder;
  private SparkLimitSwitch bottomLimitSwitch;
  private static boolean zeroed = false;
  private SparkMaxConfig mLeftConfig = new SparkMaxConfig();
  private SparkMaxConfig mRightConfig = new SparkMaxConfig();
  private SparkMaxConfig mFunnelConfig = new SparkMaxConfig();

  public ElevatorIOSparkMax() {

    mFunnelConfig.inverted(true).smartCurrentLimit(10).idleMode(IdleMode.kBrake);

    mFunnelConfig
        .absoluteEncoder
        .setSparkMaxDataPortConfig()
        .positionConversionFactor(360)
        .velocityConversionFactor(360)
        .zeroCentered(true);

    mFunnelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(FunnelConstants.kp, FunnelConstants.ki, FunnelConstants.kd)
        .positionWrappingEnabled(false);

    // base config for all motors
    mLeftConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(60)
        .closedLoopRampRate(0);

    // Create spesific right motor config from base config
    mRightConfig.apply(mLeftConfig);
    mRightConfig.follow(MechanismConstants.leftElevatorId, true);

    m_leftElevator = new SparkMax(MechanismConstants.leftElevatorId, MotorType.kBrushless);
    m_rightElevator = new SparkMax(MechanismConstants.rightElevatorId, MotorType.kBrushless);
    m_funnel = new SparkMax(MechanismConstants.funnelId, MotorType.kBrushless);
    m_belt = new SparkMax(MechanismConstants.beltId, MotorType.kBrushless);

    // Adjust left motor encoder config
    EncoderConfig encoderConfig = mLeftConfig.encoder;
    encoderConfig
        .positionConversionFactor(ElevatorConstants.positionConversion)
        .velocityConversionFactor(ElevatorConstants.velocityConversion);

    // Adjust left motor closed loop (pid controller) config
    ClosedLoopConfig closedLoopConfig = mLeftConfig.closedLoop;
    closedLoopConfig
        .pid(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false);

    SoftLimitConfig softLimitConfig = mLeftConfig.softLimit;
    softLimitConfig.forwardSoftLimit(ElevatorConstants.maxHeight).forwardSoftLimitEnabled(true);

    // Configure both motors
    m_leftElevator.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightElevator.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_funnel.configure(
        mFunnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (Constants.usePIDtuning) {
      setUpPIDTuning();
    }

    funnelEncoder = m_funnel.getAbsoluteEncoder();
    leftEncoder = m_leftElevator.getEncoder();

    funnelController = m_funnel.getClosedLoopController();
    beltController = m_belt.getClosedLoopController();
    leftElevatorController = m_leftElevator.getClosedLoopController();

    bottomLimitSwitch = m_leftElevator.getReverseLimitSwitch();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.dutyCycle = m_leftElevator.getAppliedOutput();
    inputs.appliedCurrent = m_leftElevator.getOutputCurrent();
    inputs.appliedVolts = m_leftElevator.getBusVoltage() * m_leftElevator.getAppliedOutput();
    inputs.elevatorVelo = leftEncoder.getVelocity();
    inputs.elevatorPos = leftEncoder.getPosition();
    inputs.funnelPos = funnelEncoder.getPosition();
    inputs.funnelVoltage = m_funnel.getBusVoltage() * m_funnel.getAppliedOutput();
    inputs.funnelCurrent = m_funnel.getOutputCurrent();
    inputs.isAtBottom = bottomLimitSwitch.isPressed();
    if (bottomLimitSwitch.isPressed() && zeroed == false) {
      leftEncoder.setPosition(0);
      zeroed = true;
    }

    inputs.zeroed = zeroed;

    if (Constants.usePIDtuning) {
      updatePIDTuning();
    }
  }

  public void requestElevatorPosition(double targetPostion, double feedforward) {
    if (zeroed) {
      leftElevatorController.setReference(
          targetPostion,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          feedforward,
          ArbFFUnits.kVoltage);
    }
  }

  public void requestFunnelPOS(double POS) {
    funnelController.setReference(POS, ControlType.kPosition);
  }

  public void requestBeltRPM(double RPM) {
    funnelController.setReference(RPM, ControlType.kVelocity);
  }
  /**
   * Drive motors in voltage control mode
   *
   * @param voltage Voltage to drive motors at
   */
  public void voltageControl(double voltage) {
    // clamp to -12, 12 volts
    voltage = Math.max(-12.0, Math.min(voltage, 12.0));
    leftElevatorController.setReference(voltage, ControlType.kVoltage);
  }

  public void reqestBeltVoltage(double voltage) {
    beltController.setReference(voltage, ControlType.kVoltage);
  }

  public void stop() {
    m_leftElevator.stopMotor();
    m_rightElevator.stopMotor();
  }

  private void updatePIDTuning() {
    ClosedLoopConfigAccessor closedLoop = m_leftElevator.configAccessor.closedLoop;
    MAXMotionConfigAccessor maxMotion = closedLoop.maxMotion;
    SparkMaxConfig updatedConfig = new SparkMaxConfig();
    ClosedLoopConfig CLconfig = updatedConfig.closedLoop;
    MAXMotionConfig mmConfig = CLconfig.maxMotion;

    if (SmartDashboard.getNumber("Elevator/kp", 0.0) != closedLoop.getP()) {
      CLconfig.p(SmartDashboard.getNumber("Elevator/kp", 0.0));
    }
    if (SmartDashboard.getNumber("Elevator/ki", 0.0) != closedLoop.getI()) {
      CLconfig.i(SmartDashboard.getNumber("Elevator/ki", 0.0));
    }
    if (SmartDashboard.getNumber("Elevator/kd", 0.0) != closedLoop.getD()) {
      CLconfig.d(SmartDashboard.getNumber("Elevator/kd", 0.0));
    }
    if (SmartDashboard.getNumber("Elevator/maxVelo", 0.0) != maxMotion.getMaxVelocity()) {
      mmConfig.maxVelocity(SmartDashboard.getNumber("Elevator/maxVelo", 0.0));
    }
    if (SmartDashboard.getNumber("Elevator/maxAccel", 0.0) != maxMotion.getMaxAcceleration()) {
      mmConfig.maxAcceleration(SmartDashboard.getNumber("Elevator/maxAccel", 0.0));
    }
    if (SmartDashboard.getNumber("Elevator/allowError", 0.0)
        != maxMotion.getAllowedClosedLoopError()) {
      mmConfig.allowedClosedLoopError(SmartDashboard.getNumber("Elevator/allowError", 0.0));
    }

    m_leftElevator.configure(
        updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  protected void setUpPIDTuning() {
    ClosedLoopConfigAccessor closedLoop = m_leftElevator.configAccessor.closedLoop;
    SmartDashboard.putNumber("Elevator/kp", closedLoop.getP());
    SmartDashboard.putNumber("Elevator/ki", closedLoop.getI());
    SmartDashboard.putNumber("Elevator/kd", closedLoop.getD());
    SmartDashboard.putNumber("Elevator/maxVelo", closedLoop.maxMotion.getMaxVelocity());
    SmartDashboard.putNumber("Elevator/maxAccel", closedLoop.maxMotion.getMaxAcceleration());
    SmartDashboard.putNumber(
        "Elevator/allowError", closedLoop.maxMotion.getAllowedClosedLoopError());
    SmartDashboard.putNumber("Elevator/lowerSetpoint", 0.05);
    SmartDashboard.putNumber("Elevator/upperSetpoint", 0.45);
  }
}
