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
  public static final int forwardCANrangeId = 12;

  public class ElevatorConstants {
    // Gearing constants
    public static final double elevatorGearing = 5.0;
    public static final double elevatorDrumDiam = 0.044704;
    public static final double elevatorDrumRad = elevatorDrumDiam / 2;
    public static final double positionConversion =
        (Math.PI * elevatorDrumDiam) / elevatorGearing; // revolutions -> meters
    public static final double velocityConversion = positionConversion / 60.0; // rpm -> m/s

    // Closed Loop Constant
    // public static final double kp = 10.0;
    // public static final double ki = 0.0;
    // public static final double kd = 20.0;
    // public static final double kg = 1.2208; // 1.2208 // 1.29
    // public static final double ks = 0.37251; // 0.37251 //.49
    // public static final double kv = 3.4; // 3.4 // 4.0 low
    // public static final double ka = 1.44586; // 1.44586

    public static final double kp = 12.0;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double kg = 1.1825; // 1.2208 // 1.29
    public static final double ks = 0.47; // 0.37251 //.49
    public static final double kv = 4.9; // 3.4 // 4.0 low
    public static final double ka = 0.35; // 1.44586

    public static final double simKp = 0.0;
    public static final double simKi = 0.0;
    public static final double simKd = 0.0;
    public static final double simKg = 0.55243;
    public static final double simKs = 0.0;
    public static final double simKv = 22.325; // V*s/m
    public static final double simKa = 1.54; // V*s^2/m

    public static final double maxAccel = 1.3; // m/s^1.2  max: 17.14
    public static final double maxVelo = 0.70; // m/s .75 max: .8

    public static final double simMaxAccel = 0.2; // m/s^2 max: 17.14
    public static final double simMaxVelo = 0.1; // m/s max: .8

    public static final double allowedClosedLoopError = 0.0075;

    // Elevator Constants
    public static final double intakeHeight = 0.0;
    public static final double storeAlgeaHeight = 0.03;
    public static final double storeHeight = 0.0;
    public static final double maxHeight = .589;
    public static final double levelOne = 0.05;
    public static final double levelTwo = 0.23473908007144928;
    public static final double levelThree = 0.3704996407032013;
    public static final double levelFour = 0.585;
    public static final double algeaL2 = 0.185;
    public static final double algeaL3 = 0.31171162009239197;
    public static final double algeaGround = 0.0;
    public static final double barge = 0.588;
  }

  public class WristConstants {
    public static final double gearing = 25.0 / 1.0;
    public static final double algeaGround = 20.0;

    // Closed Loop Constatns
    public static final double kp = .10;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double kg = 0.0;
    public static final double ks = 0.0;
    public static final double FF = 0.0;

    public static final double maxAccel = 40.0;
    public static final double maxVelo = 20.0;
    public static final double allowError = 2.0;

    public static final double simKp = 0.05;
    public static final double simKi = 0.0;
    public static final double simKd = 0.0;
    public static final double simKg = 0.0;
    public static final double simKs = 0.0;

    public static final double intakeCoralAngle = -2.0;
    public static final double storeAlgeaAngle = -2.0;
    public static final double storeAngle = -2.0;
    public static final double coralScoreAngle = 30.0;
    public static final double L4coralScoreAngle = 35.0;
    public static final double algeaIntakeAngle = 2.0;
    public static final double algeaScoreAngle = 2.0;
    public static final double l1angle = -45.0;
    public static final double bargeangle = -35;
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

  public class FunnelConstants {
    public static final double kp = .02;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
  }
}
