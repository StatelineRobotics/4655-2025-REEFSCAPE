



public class WristIOSim extends WristIOSparkMax {
  
  private SparkMaxSim motorSim;

  
  public WristIOSparkMax() {
    super();
    motorSim = new SparkMaxSim(m_wrist, DCMotor.getNEO(2));
  }

}
