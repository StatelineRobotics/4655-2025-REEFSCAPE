



public class WristIOSim extends WristIOSparkMax {
  
  private SparkFlexSim wristMotorSim;
  private SparkAbsoluteEncoderSim wristEncoderSim;
  private SparkMaxSim rightMotorSim;
  private SparkReletiveEncoderSim rightEncoderSim;
  private SparkMAxSim leftMotorSim;
  private SparkReletiveEncoderSim leftEncoderSim;

  
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

  private ClosedLoopConfig getWristClosedLoopConfig() {
    ClosedLoopConfig config = new ClosedLoopConfig();
    config.p(WristConstants.simKp)
          .i(WristConstatnts.simKi)
          .d(WristConstatns.simKd);
    return config;
}
