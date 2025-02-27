package frc.robot.subsystems.mechanisms.wrist;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.MAXMotionConfigAccessor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.RollerConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;

public class WristIOSparkMax implements WristIO {
  private CANrange canRange = new CANrange(MechanismConstants.canRangeID);
  private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

  protected SparkMax m_leftIntake =
      new SparkMax(MechanismConstants.leftIntakeId, MotorType.kBrushless);
  protected SparkMax m_rightIntake =
      new SparkMax(MechanismConstants.rightIntakeId, MotorType.kBrushless);
  protected SparkFlex m_wrist = new SparkFlex(MechanismConstants.wristId, MotorType.kBrushless);

  private AbsoluteEncoder wristEncoder;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkClosedLoopController leftController;
  private SparkClosedLoopController rightController;
  private SparkClosedLoopController wristController;
  private SparkMaxConfig mleftConfig = new SparkMaxConfig();
  private SparkMaxConfig mrightConfig = new SparkMaxConfig();
  private SparkFlexConfig mwristConfig = new SparkFlexConfig();
  private double RPM;
  private ClosedLoopConfig intakeConfig;

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
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(0)
        .reverseSoftLimitEnabled(false)
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

    canRangeConfig.ToFParams.withUpdateMode(UpdateModeValue.ShortRange100Hz);
    canRangeConfig
        .ProximityParams
        .withMinSignalStrengthForValidMeasurement(2500)
        .withProximityHysteresis(0.025)
        .withProximityThreshold(0.1);

    canRange.getConfigurator().apply(canRangeConfig);

    leftEncoder = m_leftIntake.getEncoder();
    rightEncoder = m_rightIntake.getEncoder();

    wristEncoder = m_wrist.getAbsoluteEncoder();

    leftController = m_leftIntake.getClosedLoopController();
    rightController = m_rightIntake.getClosedLoopController();
    wristController = m_wrist.getClosedLoopController();

    intakeConfig = mleftConfig.closedLoop;
    intakeConfig.pid(0.0013, 0, 0);

    setUpPIDTuning();
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.detectsNote = canRange.getIsDetected().getValue();

    inputs.leftIntakeRPM = leftEncoder.getVelocity();
    inputs.rightIntakeRPM = rightEncoder.getVelocity();
    inputs.wristPos = wristEncoder.getPosition();

    inputs.wristDutyCycle = m_wrist.getAppliedOutput();
    inputs.wristAppliedVoltage = m_wrist.getBusVoltage() * inputs.wristDutyCycle;
    inputs.wristAppliedCurrent = m_wrist.getOutputCurrent();

    inputs.rightDutyCycle = m_rightIntake.getAppliedOutput();
    inputs.rightAppliedVoltage = m_rightIntake.getBusVoltage() * inputs.rightDutyCycle;
    inputs.rightAppliedCurrent = m_rightIntake.getOutputCurrent();

    inputs.leftDutyCycle = m_leftIntake.getAppliedOutput();
    inputs.leftAppliedVoltage = m_leftIntake.getBusVoltage() * inputs.rightDutyCycle;
    inputs.leftAppliedCurrent = m_leftIntake.getOutputCurrent();

    if (Constants.usePIDtuning) {
      updatePIDTuning();
    }
  }

  public void requestIntakeVoltage(double voltage) {
    leftController.setReference(voltage, ControlType.kVoltage);
    rightController.setReference(voltage, ControlType.kVoltage);
  }

  public void requestWristPosition(double targetPos) {
    wristController.setReference(targetPos, ControlType.kMAXMotionPositionControl);
  }

  public void requestWristVoltage(double voltage) {
    wristController.setReference(voltage, ControlType.kVoltage);
  }

  public void requestIntakeVelo(double RPM) {
    leftController.setReference(RPM, SparkBase.ControlType.kVelocity);
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

  private void setUpPIDTuning() {
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
