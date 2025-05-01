package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import frc.robot.subsystems.MechanismConstants;

public class OutakeRollersIOTallonFX implements OutakeRollersIO {
  private final TalonFX rightMotor = new TalonFX(MechanismConstants.rightIntakeId);
  private final TalonFX leftMotor = new TalonFX(MechanismConstants.leftIntakeId);
  private final CANrange manipulatorLaser = new CANrange(MechanismConstants.forwardCANrangeId);
  private final CANrange elevatorLaser = new CANrange(MechanismConstants.canRangeID);

  public OutakeRollersIOTallonFX() {
    rightMotor.getConfigurator().apply(createConfig(InvertedValue.Clockwise_Positive));
    leftMotor.getConfigurator().apply(createConfig(InvertedValue.CounterClockwise_Positive));

    manipulatorLaser.getConfigurator().apply(createCANrangeConfig());
    elevatorLaser.getConfigurator().apply(createCANrangeConfig());
  }

  private TalonFXConfiguration createConfig(InvertedValue invert) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withInverted(invert).withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits.withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(15)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(30);
    return config;
  }

  private CANrangeConfiguration createCANrangeConfig() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ToFParams.withUpdateMode(UpdateModeValue.ShortRange100Hz);
    config.ProximityParams.withMinSignalStrengthForValidMeasurement(2500)
        .withProximityHysteresis(0.025)
        .withProximityThreshold(0.1);
    return config;
  }

  public void updateInputs(OutakeRollersIOInputs inputs) {
    inputs.elevatorLaserConnected = elevatorLaser.getIsDetected().getValue();
    inputs.elevatorLaserDetects = elevatorLaser.getIsDetected().getValue();

    inputs.manipulatorLaserConnected = manipulatorLaser.getIsDetected().getValue();
    inputs.manipulatorLaserDetects = manipulatorLaser.getIsDetected().getValue();

    inputs.rightMotorConnected = rightMotor.isConnected();
    inputs.rightMotorVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightMotorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightMotorVelocity = rightMotor.getVelocity().getValueAsDouble();

    inputs.leftMotorConnected = leftMotor.isConnected();
    inputs.leftMotorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftMotorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftMotorVelocity = leftMotor.getVelocity().getValueAsDouble();
  }

  public void requestOutakeVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public void stopOutake() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
