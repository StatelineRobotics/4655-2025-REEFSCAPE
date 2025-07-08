package frc.robot.subsystems.mechanisms.wrist;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.MAXMotionConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;

public class WristTalonFXIO implements WristIO {
  protected CANrange canRange = new CANrange(MechanismConstants.canRangeID);
  protected CANrange forwardRange = new CANrange(MechanismConstants.forwardCANrangeId);
  private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

  private TalonFX leftMotor = new TalonFX(MechanismConstants.leftIntakeId);
  private TalonFX rightMotor = new TalonFX(MechanismConstants.rightIntakeId);

  protected SparkFlex m_wrist = new SparkFlex(MechanismConstants.wristId, MotorType.kBrushless);

  private AbsoluteEncoder wristEncoder;
  private SparkClosedLoopController wristController;
  private SparkFlexConfig mwristConfig = new SparkFlexConfig();
  private double RPM;
  private ClosedLoopConfig intakeConfig;
  private ArmFeedforward armFeedforward = new ArmFeedforward(0, .25, 0);

  private LinearFilter leftFilter = LinearFilter.movingAverage(20);
  private LinearFilter rightFilter = LinearFilter.movingAverage(20);

  public WristTalonFXIO() {

    mwristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).inverted(true);

    mwristConfig
        .softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(45)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(-81);

    mwristConfig
        .absoluteEncoder
        .setSparkMaxDataPortConfig()
        .inverted(false)
        .positionConversionFactor(360)
        .velocityConversionFactor(360)
        .zeroCentered(true);

    mwristConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(WristConstants.kp)
        .i(WristConstants.ki)
        .d(WristConstants.kd)
        .positionWrappingEnabled(false);

    mwristConfig
        .closedLoop
        .maxMotion
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .maxAcceleration(WristConstants.maxAccel)
        .maxVelocity(WristConstants.maxVelo)
        .allowedClosedLoopError(WristConstants.allowError);

    leftMotor
        .getConfigurator()
        .apply(getTalonConfig().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    rightMotor
        .getConfigurator()
        .apply(getTalonConfig().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
    m_wrist.configure(mwristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    canRangeConfig.ToFParams.withUpdateMode(UpdateModeValue.ShortRange100Hz);
    canRangeConfig
        .ProximityParams
        .withMinSignalStrengthForValidMeasurement(2500)
        .withProximityHysteresis(0.025)
        .withProximityThreshold(0.1);

    canRange.getConfigurator().apply(canRangeConfig);
    forwardRange.getConfigurator().apply(canRangeConfig);

    wristEncoder = m_wrist.getAbsoluteEncoder();

    wristController = m_wrist.getClosedLoopController();

    if (Constants.usePIDtuning) {
      setUpPIDTuning();
    }
  }

  public TalonFXConfiguration getTalonConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.CurrentLimits.withStatorCurrentLimit(30)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(15)
        .withSupplyCurrentLimitEnable(true);
    config.Voltage.withPeakForwardVoltage(10).withPeakReverseVoltage(10);
    return config;
  }

  public void updateInputs(WristIOInputs inputs) {

    inputs.detectsNote = canRange.getIsDetected().getValue();
    inputs.detectsForward = forwardRange.getIsDetected().getValue();

    inputs.leftIntakeRPM = leftMotor.getVelocity(true).getValueAsDouble();
    inputs.rightIntakeRPM = rightMotor.getVelocity(true).getValueAsDouble();
    inputs.wristPos = wristEncoder.getPosition();

    inputs.wristDutyCycle = m_wrist.getAppliedOutput();
    inputs.wristAppliedVoltage = m_wrist.getBusVoltage() * inputs.wristDutyCycle;
    inputs.wristAppliedCurrent = m_wrist.getOutputCurrent();

    inputs.rightDutyCycle = rightMotor.getDutyCycle(false).getValueAsDouble();
    inputs.rightAppliedVoltage = rightMotor.getMotorVoltage(false).getValueAsDouble();
    inputs.rightAppliedCurrent = rightMotor.getStatorCurrent(true).getValueAsDouble();

    inputs.leftDutyCycle = leftMotor.getDutyCycle(false).getValueAsDouble();
    inputs.leftAppliedVoltage = leftMotor.getMotorVoltage(false).getValueAsDouble();
    inputs.leftAppliedCurrent = leftMotor.getStatorCurrent(true).getValueAsDouble();

    inputs.filteredLeftCurrent = leftFilter.calculate(inputs.leftAppliedCurrent);
    inputs.filteredRightCurrent = rightFilter.calculate(inputs.rightAppliedCurrent);

    if (Constants.usePIDtuning) {
      updatePIDTuning();
    }
  }

  public void requestIntakeVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public void requestWristPosition(double targetPos, double arbFeedforward) {
    if (Math.abs(wristEncoder.getPosition() - targetPos) > 1.0) {
      wristController.setReference(
          targetPos,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          arbFeedforward,
          ArbFFUnits.kVoltage);
    } else {
      m_wrist.stopMotor();
    }
  }

  public void requestWristVoltage(double voltage) {
    wristController.setReference(voltage, ControlType.kVoltage);
  }

  public void stop() {
    rightMotor.stopMotor();
    leftMotor.stopMotor();
    m_wrist.stopMotor();
  }

  public void stopIntake() {
    rightMotor.stopMotor();
    leftMotor.stopMotor();
  }

  public void stopWrist() {
    m_wrist.stopMotor();
  }

  protected void setUpPIDTuning() {
    ClosedLoopConfigAccessor closedLoop = m_wrist.configAccessor.closedLoop;
    SmartDashboard.putNumber("Wrist/kp", closedLoop.getP());
    SmartDashboard.putNumber("Wrist/ki", closedLoop.getI());
    SmartDashboard.putNumber("Wrist/kd", closedLoop.getD());
    SmartDashboard.putNumber("Wrist/maxVelo", closedLoop.maxMotion.getMaxVelocity());
    SmartDashboard.putNumber("Wrist/maxAccel", closedLoop.maxMotion.getMaxAcceleration());
    SmartDashboard.putNumber("Wrist/allowError", closedLoop.maxMotion.getAllowedClosedLoopError());
    SmartDashboard.putNumber("Wrist/lowerSetpoint", 0.0);
    SmartDashboard.putNumber("Wrist/upperSetpoint", 0.0);
  }

  private void updatePIDTuning() {
    ClosedLoopConfigAccessor closedLoop = m_wrist.configAccessor.closedLoop;
    MAXMotionConfigAccessor maxMotion = closedLoop.maxMotion;
    SparkFlexConfig updatedConfig = new SparkFlexConfig();
    ClosedLoopConfig CLconfig = updatedConfig.closedLoop;
    MAXMotionConfig mmConfig = CLconfig.maxMotion;

    if (SmartDashboard.getNumber("Wrist/kp", 0.0) != closedLoop.getP()) {
      CLconfig.p(SmartDashboard.getNumber("Wrist/kp", 0.0));
    }
    if (SmartDashboard.getNumber("Wrist/ki", 0.0) != closedLoop.getI()) {
      CLconfig.i(SmartDashboard.getNumber("Wrist/ki", 0.0));
    }
    if (SmartDashboard.getNumber("Wrist/kd", 0.0) != closedLoop.getD()) {
      CLconfig.d(SmartDashboard.getNumber("Wrist/kd", 0.0));
    }
    if (SmartDashboard.getNumber("Wrist/maxVelo", 0.0) != maxMotion.getMaxVelocity()) {
      mmConfig.maxVelocity(SmartDashboard.getNumber("Wrist/maxVelo", 0.0));
    }
    if (SmartDashboard.getNumber("Wrist/maxAccel", 0.0) != maxMotion.getMaxAcceleration()) {
      mmConfig.maxAcceleration(SmartDashboard.getNumber("Wrist/maxAccel", 0.0));
    }
    if (SmartDashboard.getNumber("Wrist/allowError", 0.0)
        != maxMotion.getAllowedClosedLoopError()) {
      mmConfig.allowedClosedLoopError(SmartDashboard.getNumber("Wrist/allowError", 0.0));
    }

    m_wrist.configure(
        updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
