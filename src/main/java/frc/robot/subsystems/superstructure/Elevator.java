package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.idle;

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
import frc.robot.subsystems.MechanismConstants.ElevatorConstants;
import frc.robot.subsystems.superstructure.SuperstructureController.StorePositions;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double ElevatorPosition = 0.0;
  private double FunnelPosition = 0.0;
  private double beltRPM = 0.0;

  @AutoLogOutput public Trigger atSetpoint = new Trigger(this::isAtSetpoint);

  private TrapezoidProfile profile =
      new TrapezoidProfile(new Constraints(ElevatorConstants.maxVelo, ElevatorConstants.maxAccel));
  private TrapezoidProfile.State startingState = new State();
  private TrapezoidProfile.State endState = new State();
  private static Timer timer = new Timer();
  private static double lastTime = timer.get();

  private SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second),
              Volts.of(3),
              Seconds.of(20), // Use default config
              (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
          new Mechanism((voltage) -> voltageControl(voltage.magnitude()), null, this));

  private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  private Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public Command sysIdRoutine() {
    return (sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.elevatorPos > .45))
        .andThen(holdPosition().withTimeout(0.0))
        .andThen(
            sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.elevatorPos < .15))
        .andThen(holdPosition().withTimeout(0.0))
        .andThen(
            sysIdDynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.elevatorPos > .45))
        .andThen(holdPosition().withTimeout(0.0))
        .andThen(
            sysIdDynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.elevatorPos < .15))
        .andThen(holdPosition());
  }

  private final ElevatorFeedforward feedforward;

  public Elevator(ElevatorIO io) {
    this.io = io;

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
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (DriverStation.isDisabled()) {
      io.stop();
    } else {

    }
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
    return run(() -> {
          io.voltageControl(controllerInput.getAsDouble() * -6.0 + feedforward.getKg());
        })
        .withName("Manual Run Command");
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  protected Command holdPosition() {
    return run(
        () -> {
          io.voltageControl(feedforward.getKg());
        });
  }

  private Command requestElevatorPosition(double targetPostion) {
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

  protected Command moveToSetpoint(double position) {
    return requestElevatorPosition(position);
  }

  protected Command moveToStore(BooleanSupplier hasAlgae) {
    return either(
        moveToSetpoint(StorePositions.storeAlgae.elevator),
        moveToSetpoint(StorePositions.storeCoral.elevator),
        hasAlgae);
  }

  protected Command idleCommand() {
    return idle(this);
  }

  private boolean isAtSetpoint() {
    if (Math.abs(inputs.elevatorPos - inputs.finalSetpoint)
        < ElevatorConstants.allowedClosedLoopError) {
      return true;
    }
    return false;
  }

  @AutoLogOutput
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
}
