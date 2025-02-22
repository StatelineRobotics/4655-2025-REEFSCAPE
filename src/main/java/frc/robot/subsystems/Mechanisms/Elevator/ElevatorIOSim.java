// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.elevator;

import org.opencv.features2d.Feature2D;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfigAccessor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import frc.robot.subsystems.mechanisms.MechanismConstants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getNEO(2), 
        ElevatorConstants.elevatorGearing, 
        6.80389,
        ElevatorConstants.elevatorDrumRad,
        0.0,
        Units.inchesToMeters(24.0), 
        true,
        0.0
        );

    private SparkMax m_leftElevator = new SparkMax(MechanismConstants.leftElevatorId, MotorType.kBrushless);
    private SparkMaxSim motorSim = new SparkMaxSim(m_leftElevator, DCMotor.getNEO(2));
    private SparkClosedLoopController leftElevatorController = m_leftElevator.getClosedLoopController();
    private RelativeEncoder leftEncoder = m_leftElevator.getEncoder();
    private SparkRelativeEncoderSim simEncoder = motorSim.getRelativeEncoderSim();
    private SparkLimitSwitch bottomLimitSwitch = m_leftElevator.getReverseLimitSwitch();
    private SparkLimitSwitchSim bottomLimitSwitchSim = motorSim.getReverseLimitSwitchSim();
    private static boolean zeroed = true;
    private SparkMaxConfig mLeftConfig = new SparkMaxConfig();

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(
        ElevatorConstants.simKs,
        ElevatorConstants.simKg,
        0.0
    );

    private Trigger hitBottom = new Trigger(() -> elevatorSim.hasHitLowerLimit())
            .onTrue(Commands.runOnce(() -> bottomLimitSwitchSim.setPressed(true)))
            .onFalse(Commands.runOnce(() -> bottomLimitSwitchSim.setPressed(false)));

    public ElevatorIOSim() {
        //base config for all motors
        mLeftConfig.idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(200)
                    .closedLoopRampRate(.25);

        //Adjust left motor closed loop (pid controller) config
        ClosedLoopConfig closedLoopConfig = mLeftConfig.closedLoop;
        closedLoopConfig.pid( ElevatorConstants.simKp, 
                            ElevatorConstants.simKi, 
                            ElevatorConstants.simKd)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .positionWrappingEnabled(false);

        //Adjust left motor max motion position specific config
        MAXMotionConfig maxMotionConfig = mLeftConfig.closedLoop.maxMotion;
        maxMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                        .allowedClosedLoopError(1)
                        .allowedClosedLoopError(ElevatorConstants.allowedClosedLoopError)
                        .maxAcceleration(ElevatorConstants.maxAccel)
                        .maxVelocity(ElevatorConstants.maxVelo);

        SoftLimitConfig softLimitConfig = mLeftConfig.softLimit;
        softLimitConfig.forwardSoftLimit(ElevatorConstants.maxHeight)
                    .forwardSoftLimitEnabled(true);
                        
        //Configure both motors
        m_leftElevator.configure(
            mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        if (Constants.usePIDtuning) {
            setUpPIDTuning();
        }
        
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.setInputVoltage(m_leftElevator.getBusVoltage() * m_leftElevator.getAppliedOutput());
        elevatorSim.update(.02);
        
        motorSim.iterate(
            elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.conversion_MS_RPM, 
            12, 
            0.02);
        
        inputs.dutyCycle = m_leftElevator.getAppliedOutput();
        inputs.appliedCurrent = m_leftElevator.getOutputCurrent();
        inputs.appliedVolts = m_leftElevator.getBusVoltage() * m_leftElevator.getAppliedOutput();
        inputs.elevatorPos = leftEncoder.getPosition();
        inputs.elevatorVelo = leftEncoder.getVelocity();
        inputs.zeroed = zeroed;
        if (bottomLimitSwitch.isPressed() && inputs.elevatorPos != 0) {
            leftEncoder.setPosition(0);
            zeroed = true;
        }

        if (Constants.usePIDtuning) {
            updatePIDTuning();
        }

        
    }

    private void updatePIDTuning() {
        ClosedLoopConfigAccessor closedLoop = m_leftElevator.configAccessor.closedLoop;
        MAXMotionConfigAccessor maxMotion = closedLoop.maxMotion;
        SparkMaxConfig updatedConfig = new SparkMaxConfig();
        ClosedLoopConfig CLconfig = updatedConfig.closedLoop;
        MAXMotionConfig mmConfig = CLconfig.maxMotion;

        if (SmartDashboard.getNumber("Elevator/kp",0.0) != closedLoop.getP()) {
            CLconfig.p(SmartDashboard.getNumber("Elevator/kp",0.0));
        }
        if (SmartDashboard.getNumber("Elevator/ki",0.0) != closedLoop.getI()) {
            CLconfig.i(SmartDashboard.getNumber("Elevator/ki",0.0));
        }
        if (SmartDashboard.getNumber("Elevator/kd",0.0) != closedLoop.getD()) {
            CLconfig.d(SmartDashboard.getNumber("Elevator/kd",0.0));
        }
        if (SmartDashboard.getNumber("Elevator/kg",0.0) != feedforward.getKg()) {
            //feedforward.(SmartDashboard.getNumber("Elevator/kg",0.0));
        }
        if (SmartDashboard.getNumber("Elevator/ks",0.0) != feedforward.getKs()) {
            //CLconfig.i(SmartDashboard.getNumber("Elevator/ki",0.0));
        }
        if (SmartDashboard.getNumber("Elevator/maxVelo",0.0) != maxMotion.getMaxVelocity()) {
            mmConfig.maxVelocity(SmartDashboard.getNumber("Elevator/maxVelo",0.0));
        }
        if (SmartDashboard.getNumber("Elevator/maxAccel",0.0) != maxMotion.getMaxAcceleration()) {
            mmConfig.maxAcceleration(SmartDashboard.getNumber("Elevator/maxAccel",0.0));
        }
        if (SmartDashboard.getNumber("Elevator/allowError",0.0) != maxMotion.getAllowedClosedLoopError()) {
            mmConfig.allowedClosedLoopError(SmartDashboard.getNumber("Elevator/allowError",0.0));
        }

        m_leftElevator.configure(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void setUpPIDTuning() {
        ClosedLoopConfigAccessor closedLoop = m_leftElevator.configAccessor.closedLoop;
        SmartDashboard.putNumber("Elevator/kp", closedLoop.getP());
        SmartDashboard.putNumber("Elevator/ki", closedLoop.getI());
        SmartDashboard.putNumber("Elevator/kd", closedLoop.getD());
        SmartDashboard.putNumber("Elevator/kg", feedforward.getKg());
        SmartDashboard.putNumber("Elevator/ks", feedforward.getKs());
        SmartDashboard.putNumber("Elevator/maxVelo", closedLoop.maxMotion.getMaxVelocity());
        SmartDashboard.putNumber("Elevator/maxAccel", closedLoop.maxMotion.getMaxAcceleration());
        SmartDashboard.putNumber("Elevator/allowError", closedLoop.maxMotion.getAllowedClosedLoopError());
        SmartDashboard.putNumber("Elevator/lowerSetpoint", 0.0);
        SmartDashboard.putNumber("Elevator/upperSetpoint", 0.0);
    }


    /**
     * Drive motors in maxMotion position mode
     *
     * @param targetPosition The target position for the elevator to go to in motor rotations
     * 
     */
    public void positionControl(double targetPostion) {
        if(zeroed){
            leftElevatorController.setReference(
                targetPostion, 
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                feedforward.calculate(leftEncoder.getVelocity()));
        } else {
            leftElevatorController.setReference(0, ControlType.kVoltage);
        }
    }


    /**
     * Drive motors in voltage control mode
     * 
     * @param voltage Voltage to drive motors at
     * 
     */
    public void voltageControl(double voltage) {
        voltage = voltage + ElevatorConstants.simKg;
        //clamp to -12, 12 volts
        voltage = Math.max(-12.0, Math.min(voltage, 12.0));
        leftElevatorController.setReference(voltage, ControlType.kVoltage);
    }


    /**
     * Stop both motors
     */
    public void stop() {
        m_leftElevator.stopMotor();
    }

}
