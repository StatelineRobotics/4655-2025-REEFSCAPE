package frc.robot.subsystems.mechanisms.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.MAXMotionConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.MechanismConstants;

import frc.robot.subsystems.mechanisms.MechanismConstants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {

  private SparkMax m_funnel;
  private SparkMax m_belt;

  private SparkClosedLoopController funnelController;
  private SparkClosedLoopController beltController;

  private RelativeEncoder funnelEncoder;
  private SparkLimitSwitch limitSwitch;
  
  private SparkMax m_leftElevator = new SparkMax(MechanismConstants.leftElevatorId, MotorType.kBrushless);
  private SparkMax m_rightElevator = new SparkMax(MechanismConstants.rightElevatorId, MotorType.kBrushless);
  private SparkClosedLoopController leftElevatorController = m_leftElevator.getClosedLoopController();
  private RelativeEncoder leftEncoder = m_leftElevator.getEncoder();
  private SparkLimitSwitch bottomLimitSwitch = m_leftElevator.getReverseLimitSwitch();
  private static boolean zeroed;
  private SparkMaxConfig mLeftConfig = new SparkMaxConfig();
  private SparkMaxConfig mRightConfig = new SparkMaxConfig();

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(
                                            ElevatorConstants.ks,
                                            ElevatorConstants.kg,
                                            0.0);


  public ElevatorIOSparkMax() {
    //base config for all motors
    mLeftConfig.idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(60);
    
    //Create spesific right motor config from base config
    mRightConfig.apply(mLeftConfig);
    mRightConfig.follow(MechanismConstants.leftElevatorId,true);

    
    m_leftElevator = new SparkMax(MechanismConstants.leftElevatorId, MotorType.kBrushless);
    m_rightElevator = new SparkMax(MechanismConstants.rightElevatorId, MotorType.kBrushless);
    m_funnel = new SparkMax(MechanismConstants.funnelId, MotorType.kBrushless);
    m_belt = new SparkMax(MechanismConstants.funnelId, MotorType.kBrushed);


    //Adjust left motor encoder config
    EncoderConfig encoderConfig = mLeftConfig.encoder;

    //Adjust left motor closed loop (pid controller) config
    ClosedLoopConfig closedLoopConfig = mLeftConfig.closedLoop;
    closedLoopConfig.pid(ElevatorConstants.kp, 
                          ElevatorConstants.ki, 
                          ElevatorConstants.kd)
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .positionWrappingEnabled(false);

    //Adjust left motor max motion position specific config
    MAXMotionConfig maxMotionConfig = mLeftConfig.closedLoop.maxMotion;
    maxMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                    .allowedClosedLoopError(ElevatorConstants.allowedClosedLoopError)
                    .maxAcceleration(ElevatorConstants.maxAccel)
                    .maxVelocity(ElevatorConstants.maxVelo);
    
    SoftLimitConfig softLimitConfig = mLeftConfig.softLimit;
    softLimitConfig.forwardSoftLimit(ElevatorConstants.maxHeight)
                    .forwardSoftLimitEnabled(true);
                    
    //Configure both motors
    m_leftElevator.configure(
        mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightElevator.configure(
        mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (Constants.usePIDtuning) {
      setUpPIDTuning();
    }
    funnelEncoder = m_funnel.getEncoder();

    funnelController = m_funnel.getClosedLoopController();
    beltController = m_belt.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.dutyCycle = m_leftElevator.getAppliedOutput();
    inputs.appliedCurrent = m_leftElevator.getOutputCurrent();
    inputs.appliedVolts = m_leftElevator.getBusVoltage() * m_leftElevator.getAppliedOutput();
    inputs.elevatorVelo = leftEncoder.getVelocity();
    inputs.elevatorPos = leftEncoder.getPosition();
    if (bottomLimitSwitch.isPressed()) {
      leftEncoder.setPosition(0);
      zeroed = true;
    }

    inputs.zeroed = zeroed;

    if (Constants.usePIDtuning) {
      updatePIDTuning();
    }
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
        feedforward.calculate(leftEncoder.getVelocity()),
        ArbFFUnits.kVoltage);
    } else {
      leftElevatorController.setReference(-1, ControlType.kVoltage);
    }
  }

  public void requestFunnelPOS(double POS){
    funnelController.setReference(POS, ControlType.kPosition);
  }

  public void requestBeltRPM(double RPM){
    funnelController.setReference(RPM, ControlType.kVelocity);
  }
  /**
   * Drive motors in voltage control mode
   * 
   * @param voltage Voltage to drive motors at
   * 
   */
  public void voltageControl(double voltage) {
    voltage = voltage + ElevatorConstants.kg;
    //clamp to -12, 12 volts
    voltage = Math.max(-12.0, Math.min(voltage, 12.0));
    leftElevatorController.setReference(voltage, ControlType.kVoltage);
  }


  /**
   * Stop both motors
   */
  public void stop() {
    m_leftElevator.stopMotor();
    m_rightElevator.stopMotor();
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
}

