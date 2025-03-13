package frc.robot.subsystems.mechanisms;

public class MechanismConstants {

  // Motor Constants
  public static final int leftElevatorId = 3;
  public static final int rightElevatorId = 4;
  public static final int leftIntakeId = 5;
  public static final int rightIntakeId = 6;
  public static final int wristId = 7;
  public static final int climberId = 9;
  public static final int funnelId = 8;
  public static final int beltId = 10;
  public static final int canRangeID = 2;
  public static final int CANdleID = 11;

  public class ElevatorConstants {
    // Gearing constants
    public static final double elevatorGearing = 25.0;
    public static final double elevatorDrumDiam = 0.044704;
    public static final double elevatorDrumRad = elevatorDrumDiam / 2;
    public static final double conversion_MS_RPM =
        (60.0 * elevatorGearing) / (Math.PI * elevatorDrumDiam);
    public static final double conversion_RPM_MS = 1.0 / conversion_MS_RPM;
    public static final double conversion_M_Rot = (elevatorGearing) / (Math.PI * elevatorDrumDiam);
    public static final double conversion_Rot_M = 1.0 / conversion_M_Rot;

    // Closed Loop Constant
    public static final double kp = 0.06;
    public static final double ki = 0.0;
    public static final double kd = 0.02;
    public static final double kg = 0.35;
    public static final double ks = 0.0;

    public static final double simKp = .04;
    public static final double simKi = 0.0;
    public static final double simKd = 0.0;
    public static final double simKg = 0.1375;
    public static final double simKs = 0.0;

    public static final double maxAccel = 10000.0;
    public static final double maxVelo = 4000.0;

    public static final double simMaxAccel = 20000;
    public static final double simMaxVelo = 5000;

    public static final double allowedClosedLoopError = 0.5;

    // Elevator Constants
    public static final double intakeHeight = 0.5;
    public static final double storeAlgeaHeight = 11.0;
    public static final double storeHeight = 0.0;
    public static final double maxHeight = 104;
    public static final double levelOne = 30;
    public static final double levelTwo = 40;
    public static final double levelThree = 65;
    public static final double levelFour = 104;
    public static final double algeaL2 = 35;
    public static final double algeaL3 = 60;
    public static final double algeaGround = 0.0;
  }

  public class WristConstants {
    public static final double gearing = 25.0 / 1.0;
    public static final double algeaGround = 30.0;

    // Closed Loop Constatns
    public static final double kp = 0.05;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double kg = 0.0;
    public static final double ks = 0.0;

    public static final double maxAccel = 40.0;
    public static final double maxVelo = 20.0;
    public static final double allowError = 1.0;

    public static final double simKp = 0.05;
    public static final double simKi = 0.0;
    public static final double simKd = 0.0;
    public static final double simKg = 0.0;
    public static final double simKs = 0.0;

    public static final double intakeCoralAngle = 3.0;
    public static final double storeAlgeaAngle = 2.0;
    public static final double storeAngle = 3.0;
    public static final double coralScoreAngle = 30.0;
    public static final double algeaIntakeAngle = 30.0;
    public static final double algeaScoreAngle = 2.0;
  }

  public class RollerConstants {
    public static final int currentLimit = 10;

    public static final double kp = 0.0;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double ff = 0.0;
  }

  public class ClimberConstants {
    public static final double kg = 0.5;
    public static final double ks = 0.0;
    public static final double ka = 0.0;
    public static final int climberCurrentLimit = 10;
  }
}
