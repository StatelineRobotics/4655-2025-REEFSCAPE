package frc.robot.subsystems.mechanisms;

public class MechanismConstants {

  // Motor Constants
  public static final int leftElevatorId = 0;
  public static final int rightElevatorId = 0;
  public static final int leftIntakeId = 0;
  public static final int rightIntakeId = 0;
  public static final int wristId = 0;
  public static final int climberId = 0;
  public static final int funnelId = 0;
  public static final int beltId = 0;


  public class ElevatorConstants {
    //Gearing constants
    public static final double elevatorGearing = 25.0;
    public static final double elevatorDrumDiam = 0.044704;
    public static final double elevatorDrumRad = elevatorDrumDiam / 2;
    public static final double conversion_MS_RPM = (60.0 * elevatorGearing)/(Math.PI * elevatorDrumDiam);
    public static final double conversion_RPM_MS = 1.0 / conversion_MS_RPM;
    public static final double conversion_M_Rot = (elevatorGearing)/(Math.PI * elevatorDrumDiam);
    public static final double conversion_Rot_M = 1.0 / conversion_M_Rot;

    //Closed Loop Constants
    public static final double kp = 0.0;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double kg = 0.0;
    public static final double ks = 0.0;
    
    // Elevator Constants
    public static final double levelOne = 0;
    public static final double levelTwo = 0;
    public static final double levelThree = 0;
    // public final double levelFour = 0;

  }

}
