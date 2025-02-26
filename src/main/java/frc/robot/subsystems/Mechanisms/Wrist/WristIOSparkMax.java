package frc.robot.subsystems.mechanisms.wrist;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.RollerConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;
import frc.robot.subsystems.mechanisms.wrist.WristIO.WristIOInputs;
import java.util.function.Supplier;

public class WristIOSparkMax implements WristIO {
  // private CANrange canRange = new CANrange(MechanismConstants.canRangeID);
  private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
  // public Trigger detectsCoral = new Trigger(() -> canRange.getIsDetected().getValue());
  private SparkMax m_leftIntake =
      new SparkMax(MechanismConstants.leftIntakeId, MotorType.kBrushless);
  private SparkMax m_rightIntake =
      new SparkMax(MechanismConstants.rightIntakeId, MotorType.kBrushless);
  private SparkFlex m_wrist = new SparkFlex(MechanismConstants.wristId, MotorType.kBrushless);
  private AbsoluteEncoder wristEncoder = m_wrist.getAbsoluteEncoder();
  private RelativeEncoder leftEncoder = m_leftIntake.getEncoder();
  private RelativeEncoder rightEncoder = m_rightIntake.getEncoder();
  private SparkClosedLoopController leftController = m_leftIntake.getClosedLoopController();
  private SparkClosedLoopController rightController = m_rightIntake.getClosedLoopController();
  private SparkClosedLoopController wristController = m_wrist.getClosedLoopController();
  private SparkMaxConfig mleftConfig = new SparkMaxConfig();
  private SparkMaxConfig mrightConfig = new SparkMaxConfig();
  private SparkFlexConfig mwristConfig = new SparkFlexConfig();
  private ArmFeedforward feedforward = new ArmFeedforward(WristConstants.ks, WristConstants.kg, 0);
  private double RPM;

  public WristIOSparkMax() {
    mleftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(10).inverted(false);

    mleftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(RollerConstants.kp)
        .i(RollerConstants.ki)
        .d(RollerConstants.kd)
        .velocityFF(RollerConstants.ff);

    mrightConfig.apply(mleftConfig).inverted(true);

    mwristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

    mwristConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(-1)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(-1);

    mwristConfig
        .absoluteEncoder
        .setSparkMaxDataPortConfig()
        .inverted(false)
        .positionConversionFactor(Math.PI * 2.0)
        .velocityConversionFactor(Math.PI * 2.0);

    mwristConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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

    m_leftIntake.configure(
        mleftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightIntake.configure(
        mrightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wrist.configure(mwristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    canRangeConfig
        .ToFParams
        .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
        .withUpdateFrequency(50);
    // canRange.getConfigurator().apply(canRangeConfig);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.leftIntakeRPM = leftEncoder.getVelocity();
    inputs.rightIntakeRPM = rightEncoder.getVelocity();
    inputs.wristPos = wristEncoder.getPosition();
  }

  public void requestIntakeVoltage(double voltage) {
    leftController.setReference(voltage, ControlType.kVoltage);
    rightController.setReference(voltage, ControlType.kVoltage);
  }

  public void requestWristPosition(double targetPos) {
    wristController.setReference(
        targetPos,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(wristEncoder.getPosition(), wristEncoder.getVelocity()),
        ArbFFUnits.kVoltage);
  }

  public void requestWristVoltage(double voltage) {
    wristController.setReference(
        voltage + feedforward.calculate(wristEncoder.getPosition(), wristEncoder.getVelocity()),
        ControlType.kVoltage);
  }

  public void requestIntakeVelo(double RPM) {
    leftController.setReference(RPM, SparkBase.ControlType.kVelocity);
    rightController.setReference(RPM, SparkBase.ControlType.kVelocity);
  }

  public void stop() {
    m_leftIntake.stopMotor();
    m_rightIntake.stopMotor();
    m_wrist.stopMotor();
  }

  public void stopIntake() {
    m_leftIntake.stopMotor();
    m_rightIntake.stopMotor();
  }

  public void stopWrist() {
    m_wrist.stopMotor();
  }

  public Supplier<Boolean> detectCoral() {
    // return canRange.getIsDetected().asSupplier();
    return () -> false;
  }
}
