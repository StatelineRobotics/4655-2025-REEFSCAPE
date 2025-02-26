package frc.robot.subsystems.mechanisms.wrist;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.mechanisms.MechanismConstants.WristConstants;

public class WristIOSim extends WristIOSparkMax {

  private SparkFlexSim wristMotorSim;
  private SparkAbsoluteEncoderSim wristEncoderSim;
  private SparkMaxSim rightMotorSim;
  private SparkRelativeEncoderSim rightEncoderSim;
  private SparkMaxSim leftMotorSim;
  private SparkRelativeEncoderSim leftEncoderSim;
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNeoVortex(1),
          45,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), Units.lbsToKilograms(12)),
          Units.inchesToMeters(8),
          Math.toRadians(-45),
          Math.toRadians(0),
          false, // it can hold its own weight so effectively no gravity
          0);
  private FlywheelSim leftSim =
      new FlywheelSim(
          // idk what the MOI is
          LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 1, 25), DCMotor.getNeo550(1));

  private FlywheelSim rightSim =
      new FlywheelSim(
          // idk what the MOI is
          LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 1, 25), DCMotor.getNeo550(1));

  public WristIOSim() {
    super();

    wristMotorSim = new SparkFlexSim(m_wrist, DCMotor.getNeoVortex(1));
    wristEncoderSim = wristMotorSim.getAbsoluteEncoderSim();
    SparkFlexConfig wConfig = new SparkFlexConfig();
    wConfig.apply(getWristClosedLoopConfig());
    m_wrist.configure(wConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    leftMotorSim = new SparkMaxSim(m_leftIntake, DCMotor.getNeo550(1));
    leftEncoderSim = leftMotorSim.getRelativeEncoderSim();

    rightMotorSim = new SparkMaxSim(m_rightIntake, DCMotor.getNeo550(1));
    rightEncoderSim = rightMotorSim.getRelativeEncoderSim();
  }

  public void updateInputs(WristIOInputs inputs) {
    super.updateInputs(inputs);
    updateSim(inputs);
  }

  private void updateSim(WristIOInputs inputs) {
    leftSim.setInputVoltage(inputs.leftAppliedVoltage);
    rightSim.setInputVoltage(inputs.rightAppliedVoltage);
    armSim.setInputVoltage(inputs.wristAppliedVoltage);
  }

  private ClosedLoopConfig getWristClosedLoopConfig() {
    ClosedLoopConfig config = new ClosedLoopConfig();
    config.p(WristConstants.simKp).i(WristConstants.simKi).d(WristConstants.simKd);
    return config;
  }
}
