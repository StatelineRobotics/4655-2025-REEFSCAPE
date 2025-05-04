package frc.robot.subsystems.superstructure;

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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.MechanismConstants;
import frc.robot.subsystems.MechanismConstants.WristConstants;

public class WristIOSparkMax implements WristIO {
  protected SparkFlex m_wrist = new SparkFlex(MechanismConstants.wristId, MotorType.kBrushless);

  private AbsoluteEncoder wristEncoder;

  private SparkClosedLoopController wristController;

  private SparkFlexConfig mwristConfig = new SparkFlexConfig();
  private double RPM;
  private ClosedLoopConfig intakeConfig;

  private LinearFilter leftFilter = LinearFilter.movingAverage(10);
  private LinearFilter rightFilter = LinearFilter.movingAverage(10);

  public WristIOSparkMax() {

    mwristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30);

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
        .positionConversionFactor(360)
        .velocityConversionFactor(360)
        .zeroCentered(true);

    mwristConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(WristConstants.kp)
        .i(WristConstants.ki)
        .d(WristConstants.kd)
        .velocityFF(WristConstants.FF)
        .positionWrappingEnabled(false);

    mwristConfig
        .closedLoop
        .maxMotion
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .maxAcceleration(WristConstants.maxAccel)
        .maxVelocity(WristConstants.maxVelo)
        .allowedClosedLoopError(WristConstants.allowError);

    m_wrist.configure(mwristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristEncoder = m_wrist.getAbsoluteEncoder();

    wristController = m_wrist.getClosedLoopController();

    intakeConfig.pid(0.0013, 0, 0);

    if (Constants.usePIDtuning) {
      setUpPIDTuning();
    }
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.wristPos = wristEncoder.getPosition();

    inputs.wristDutyCycle = m_wrist.getAppliedOutput();
    inputs.wristAppliedVoltage = m_wrist.getBusVoltage() * inputs.wristDutyCycle;
    inputs.wristAppliedCurrent = m_wrist.getOutputCurrent();

    if (Constants.usePIDtuning) {
      updatePIDTuning();
    }
  }

  public void requestWristPosition(double targetPos, double arbFeedforward) {
    wristController.setReference(
        targetPos,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        arbFeedforward,
        ArbFFUnits.kVoltage);
  }

  public void requestWristVoltage(double voltage) {
    wristController.setReference(voltage, ControlType.kVoltage);
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
