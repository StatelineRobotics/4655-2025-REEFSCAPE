// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.elevator;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.MechanismConstants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorIOSim extends ElevatorIOSparkMax {
  ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          ElevatorConstants.elevatorGearing,
          27.2155,
          ElevatorConstants.elevatorDrumRad,
          0.0,
          Units.inchesToMeters(24.0),
          true,
          0.0);

  private SparkMaxSim motorSim = new SparkMaxSim(m_leftElevator, DCMotor.getNEO(2));

  private SparkRelativeEncoderSim simEncoder = motorSim.getRelativeEncoderSim();

  private SparkLimitSwitchSim bottomLimitSwitchSim = motorSim.getReverseLimitSwitchSim();
  private SparkMaxConfig simConfig = new SparkMaxConfig();

  public ElevatorIOSim() {
    super();

    bottomLimitSwitchSim.setPressed(true);

    // Adjust left motor closed loop (pid controller) config
    ClosedLoopConfig closedLoopConfig = simConfig.closedLoop;
    closedLoopConfig
        .pid(ElevatorConstants.simKp, ElevatorConstants.simKi, ElevatorConstants.simKd)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(false);

    // Configure both motors
    m_leftElevator.configure(
        simConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    if (Constants.usePIDtuning) {
      super.setUpPIDTuning();
    }
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInputVoltage(m_leftElevator.getBusVoltage() * m_leftElevator.getAppliedOutput());
    elevatorSim.update(.02);

    motorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), 12, 0.02);
    // motorSim.setPosition(elevatorSim.getPositionMeters());

    Logger.recordOutput("Elevator/simHeight", elevatorSim.getPositionMeters());
    Logger.recordOutput("Elevator/simVelo", elevatorSim.getVelocityMetersPerSecond());

    bottomLimitSwitchSim.setPressed(elevatorSim.wouldHitLowerLimit(inputs.elevatorPos));

    super.updateInputs(inputs);
  }
}
