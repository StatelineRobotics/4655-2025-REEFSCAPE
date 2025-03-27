package frc.robot.subsystems.mechanisms.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.subsystems.mechanisms.MechanismConstants.ElevatorConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double ElevatorPosition = 0.0;
  private double FunnelPosition = 0.0;
  private double beltRPM = 0.0;
  
  private TrapezoidProfile profile =
      new TrapezoidProfile(new Constraints(ElevatorConstants.maxVelo, ElevatorConstants.maxAccel));
  private TrapezoidProfile.State startingState = new State();
  private TrapezoidProfile.State endState = new State();
  private static Timer timer = new Timer();
  private static double lastTime = timer.get();

  SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second),
              Volts.of(3),
              Seconds.of(20), // Use default config
              (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
          new Mechanism((voltage) -> voltageControl(voltage.magnitude()), null, this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public Command sysIdRoutine() {
    return (sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.elevatorPos > .45))
        .andThen(run(() -> voltageControl(0.0)).withTimeout(0.0))
        .andThen(
            sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.elevatorPos < .15))
        .andThen(run(() -> voltageControl(0.0)).withTimeout(0.0))
        .andThen(
            sysIdDynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.elevatorPos > .45))
        .andThen(run(() -> voltageControl(0.0)).withTimeout(0.0))
        .andThen(
            sysIdDynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.elevatorPos < .15))
        .andThen(() -> voltageControl(0.0));
  }

  private final ElevatorFeedforward feedforward;

  public Elevator(ElevatorIO io) {
    this.io = io;
    if (true) {
      SmartDashboard.putData("Elevator/lowerTestCommand", testLowerPosition());
      SmartDashboard.putData("Elevator/upperTestCommand", testUpperPosition());
    }

    if (Robot.isSimulation()) {
      feedforward =
          new ElevatorFeedforward(
              ElevatorConstants.simKs,
              ElevatorConstants.simKg,
              ElevatorConstants.simKv,
              ElevatorConstants.simKa);
    } else {
      feedforward =
          new ElevatorFeedforward(
              ElevatorConstants.ks,
              ElevatorConstants.kg,
              ElevatorConstants.kv,
              ElevatorConstants.ka);
    }
    SmartDashboard.putData("sysID", sysIdRoutine());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TestArmAngle", 22.5);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (DriverStation.isDisabled()) {
      io.stop();
    } else {

    }
  }

  public void requestBeltRPM(double RPM) {
    beltRPM = RPM;
  }

  public void reqestBeltVoltage(double voltage) {
    io.reqestBeltVoltage(voltage);
  }

  public void requestFunnelPOS(double POS) {
    io.requestFunnelPOS(POS);
  }

  private void stop() {
    io.stop();
  }

  private void voltageControl(double voltage) {
    io.voltageControl(voltage);
  }

  private void positionControl(double targetPostion, double feedforward) {
    inputs.setpoint = targetPostion;
    io.requestElevatorPosition(targetPostion, feedforward);
  }

  public Command manualRunCommand(DoubleSupplier controllerInput) {
    return this.run(
            () -> {
              voltageControl(controllerInput.getAsDouble() * -6.0 + 1.0);
            })
        .withName("Maual Run Command");
  }

  public Command stopCommand() {
    return this.runOnce(this::stop);
  }

  public Command holdPosition() {
    return this.run(
        () -> {
          voltageControl(1.0);
        });
  }

  public Command testLowerPosition() {
    return goToPositionCommand(SmartDashboard.getNumber("Elevator/lowerSetpoint", 0.05));
  }

  public Command testUpperPosition() {
    return goToPositionCommand(SmartDashboard.getNumber("Elevator/upperSetpoint", 0.45));
  }

  public Command goToPositionCommand(double targetPostion) {
    return startRun(
        () -> {
          inputs.finalSetpoint = targetPostion;
          startingState = new State(inputs.elevatorPos, inputs.elevatorVelo);
          endState = new State(targetPostion, 0.0);
          timer.restart();
        },
        () -> {
          double currentTime = timer.get();
          Logger.recordOutput("time", currentTime);
          // TrapezoidProfile.State currentState = new State(inputs.elevatorPos,
          // inputs.elevatorVelo);
          State currentTarget = profile.calculate(currentTime, startingState, endState);
          State nextState = profile.calculate(currentTime + 0.02, startingState, endState);
          double voltageFeed =
              feedforward.calculateWithVelocities(currentTarget.velocity, nextState.velocity);
          positionControl(currentTarget.position, voltageFeed);
          inputs.veolocitySetpoint = currentTarget.velocity;
          inputs.motionSetpoint = currentTarget.position;
          lastTime = currentTime;
        });
  }

  public Command goToAnyPositionCommand(double targetPostion) {
    return defer(() -> goToAnyPositionCommand(targetPostion));
  }

  public boolean isAtSetpoint() {
    if (Math.abs(inputs.elevatorPos - inputs.finalSetpoint)
        < ElevatorConstants.allowedClosedLoopError) {
      return true;
    }
    return false;
  }

  public boolean isHomed() {
    return inputs.zeroed;
  }

  @AutoLogOutput
  public double getCarrageHeight() {
    return inputs.elevatorPos * 3.0;
  }

  @AutoLogOutput
  public double get2ndStageHeight() {
    return (inputs.elevatorPos * 2.0) - Units.inchesToMeters(1.0);
  }

  @AutoLogOutput
  public double get1stStageHeight() {
    return inputs.elevatorPos - Units.inchesToMeters(2.0);
  }

  @AutoLogOutput
  public double getCarrageVelocity() {
    return inputs.elevatorVelo * 3.0;
  }

  @AutoLogOutput
  public double getVelocity() {
    return inputs.elevatorVelo;
  }

  @AutoLogOutput
  public double getFunnelPos() {
    return inputs.funnelPos;
  }

  public Command getL1Command() {
    return goToPositionCommand(ElevatorConstants.levelOne);
  }

  public Command getL2Command() {
    return goToPositionCommand(ElevatorConstants.levelTwo);
  }

  public Command getL3Command() {
    return goToPositionCommand(ElevatorConstants.levelThree);
  }

  public Command getL4Command() {
    return goToPositionCommand(ElevatorConstants.levelFour);
  }

  public Command getAlgaeL2Command() {
    return goToPositionCommand(ElevatorConstants.algeaL2);
  }

  public Command getAlgaeL3Command() {
    return goToPositionCommand(ElevatorConstants.algeaL3);
  }

  public Command getIntakeCommand() {
    return goToPositionCommand(ElevatorConstants.intakeHeight);
  }

  public Command getAlgaeIntakeCommand() {
    return goToPositionCommand(ElevatorConstants.algeaGround);
  }

  public Command getCoralStoreCommand() {
    return goToPositionCommand(ElevatorConstants.storeHeight);
  }

  public Command getAlgaeStoreCommand() {
    return goToPositionCommand(ElevatorConstants.storeAlgeaHeight);
  }
}
