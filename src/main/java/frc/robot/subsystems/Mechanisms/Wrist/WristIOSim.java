



public class WristIOSim extends WristIOSparkMax {
  
  private SparkFlexSim wristMotorSim;
  private SparkAbsoluteEncoderSim wristEncoderSim;
  private SparkMaxSim rightMotorSim;
  private SparkReletiveEncoderSim rightEncoderSim;
  private SparkMAxSim leftMotorSim;
  private SparkReletiveEncoderSim leftEncoderSim;
  private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getVortex(1), 
                                                           45, 
                                                           estimateMOI(inchesToMeters(8), lbTokg(12), 
                                                           inchesToMeters(8), 
                                                           Math.degreesToRad(-45), 
                                                           Math.degreesToRad(0), 
                                                           false,//it can hold its own weight so effectively no gravity 
                                                           0);
  private FlywheelSim leftSim = new FlyweelSim(
  //idk what the MOI is
                                    LinearSystemId.createFlywheelSystem(DCmotor.getNeo550(1), 1, double, 25),
                                    DCMotor.getNeo550(1));
  
  private FlywheelSim rightSim = new FlyweelSim(
  //idk what the MOI is
                                    LinearSystemId.createFlywheelSystem(DCmotor.getNeo550(1), 1, double, 25),
                                    DCMotor.getNeo550(1));

  
  public WristIOSparkMax() {
    super();
    
    wristMotorSim = new SparkFlexSim(m_wrist, DCMotor.getVotex(1));
    wristEncoderSim = wristMotorSim.getAbsoluteEncoderSim();
    SparkFlexConfig wConfig = new SparkFlexConfig();
    wConfig.apply(getWristClosedLoopConfig)
    m_wrist.configure(wConfig, noReset, noPersist);

    leftMotorSim = new SparkMaxSim(m_leftIntake, DCMotor.getNeo550(1));
    leftEncoderSim = leftMotorSim.getEncoderSim();

    rightMotorSim = new SparkMaxSim(m_rightIntake, DCMotor.getNeo550(1));
    rightEncoderSim = rightMotorSim.getEncoderSim();
    
  }

  private void updateSim(wristInputs inputs) {
    leftSim.setInputVoltage(inputs.leftVoltage);
    rightSim.setInputsVoltage(inputs.rightVoltage);
    armSim.setInputVoltage(inputs.wristVoltage);
  }

  private ClosedLoopConfig getWristClosedLoopConfig() {
    ClosedLoopConfig config = new ClosedLoopConfig();
    config.p(WristConstants.simKp)
          .i(WristConstatnts.simKi)
          .d(WristConstatns.simKd);
    return config;
}
