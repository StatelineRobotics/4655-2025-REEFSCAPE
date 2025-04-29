package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.subsystems.MechanismConstants;
import frc.robot.subsystems.MechanismConstants.FunnelConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class HopperIOSparkMax implements HopperIO {

  public SparkMax pivotMotor = new SparkMax(MechanismConstants.funnelId, MotorType.kBrushless);
  public SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  public SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  public SparkMax beltMotor = new SparkMax(MechanismConstants.beltId, MotorType.kBrushless);
  public RelativeEncoder beltEncoder = beltMotor.getEncoder();

  public HopperIOSparkMax() {
    SparkMaxConfig wristConfig = new SparkMaxConfig();

    wristConfig.inverted(false).smartCurrentLimit(20);
    wristConfig
        .closedLoop
        .pid(FunnelConstants.kp, FunnelConstants.ki, FunnelConstants.kd)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristConfig
        .absoluteEncoder
        .setSparkMaxDataPortConfig()
        .positionConversionFactor(360)
        .velocityConversionFactor(360)
        .zeroCentered(true);

    SparkMaxConfig beltConfig = new SparkMaxConfig();
    beltConfig.inverted(false).smartCurrentLimit(10);
  }

  public void updateInputs(HopperIOInputs inputs) {
    inputs.pivotAngle = pivotEncoder.getPosition();
    inputs.pivotMotorVoltage = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotMotorCurrent = pivotMotor.getOutputCurrent();

    inputs.beltMotorVoltage = beltMotor.getBusVoltage() * beltMotor.getAppliedOutput();
    inputs.beltMotorCurrent = beltMotor.getOutputCurrent();
  }

  public void requestBeltVoltage(double voltage) {
    beltMotor.setVoltage(voltage);
  }

  public void requestPivotAngle(double angle) {
    pivotController.setReference(angle, ControlType.kPosition);
  }

  public void stopBelt() {
    beltMotor.stopMotor();
  }

  public void stopWrist() {
    pivotMotor.stopMotor();
  }
}
