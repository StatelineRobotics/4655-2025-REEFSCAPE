package frc.robot.subsystems.mechanisms.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import frc.robot.subsystems.mechanisms.wrist.WristIO.WristIOInputs;

public class WristIOSparkMax {
  private SparkMax m_leftIntake;
  private SparkMax m_rightIntake;
  private SparkMax m_wrist;
  private RelativeEncoder wristEncoder;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkClosedLoopController leftController;
  private SparkClosedLoopController rightController;
  private SparkClosedLoopController wristController;
  private SparkBaseConfig mleftConfig;
  private SparkBaseConfig mrightConfig;
  private SparkBaseConfig mwristConfig;
  private double RPM;

  public WristIOSparkMax() {
    mleftConfig.idleMode(IdleMode.kCoast);
    mleftConfig.smartCurrentLimit(20);
    mrightConfig = mleftConfig;
    mwristConfig = mleftConfig;
    mwristConfig.idleMode(IdleMode.kBrake);

    m_leftIntake = new SparkMax(MechanismConstants.leftIntakeId, MotorType.kBrushless);
    m_rightIntake = new SparkMax(MechanismConstants.rightIntakeId, MotorType.kBrushless);
    m_wrist = new SparkMax(0, MotorType.kBrushless);
  

    m_leftIntake.configure(
        mleftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightIntake.configure(
        mrightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wrist.configure(mwristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristEncoder = m_wrist.getEncoder();
    leftEncoder = m_leftIntake.getEncoder();
    rightEncoder = m_leftIntake.getEncoder();

    leftController = m_leftIntake.getClosedLoopController();
    rightController = m_rightIntake.getClosedLoopController();
    wristController = m_wrist.getClosedLoopController();
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.leftIntakeRPM = leftEncoder.getVelocity();
    inputs.rightIntakeRPM = rightEncoder.getVelocity();
    inputs.wristPos = wristEncoder.getPosition();
  }

  public void requestWristPOS(double POS) {
    wristController.setReference(POS, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public void requestIntake(double RPM){
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
}
