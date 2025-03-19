package frc.robot.subsystems.mechanisms.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.mechanisms.MechanismConstants.ElevatorConstants;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double ElevatorPosition = 0.0;
  private double FunnelPosition = 0.0;
  private double beltRPM = 0.0;

  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  private final ElevatorFeedforward feedforward;

  ProfiledPIDController profiledPIDController =
      new ProfiledPIDController(
          ElevatorConstants.simKp,
          ElevatorConstants.ki,
          ElevatorConstants.kd,
          new Constraints(ElevatorConstants.simMaxVelo, ElevatorConstants.simMaxAccel));

  @AutoLogOutput public Trigger atSetpoint = new Trigger(() -> profiledPIDController.atGoal());

  public Elevator(ElevatorIO io) {
    this.io = io;
    if (Constants.usePIDtuning) {
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
      feedforward = new ElevatorFeedforward(0, 0, 0);
    }
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
              voltageControl(controllerInput.getAsDouble() * -6.0);
            })
        .withName("Maual Run Command");
  }

  public Command stopCommand() {
    return this.runOnce(this::stop);
  }

  public Command holdPosition() {
    return this.run(
        () -> {
          voltageControl(ElevatorConstants.simKg);
        });
  }

  public Command testLowerPosition() {
    return defer(
        () -> goToPositionCommand(SmartDashboard.getNumber("Elevator/lowerSetpoint", 0.0)));
  }

  public Command testUpperPosition() {
    return defer(
        () -> goToPositionCommand(SmartDashboard.getNumber("Elevator/upperSetpoint", 0.0)));
  }

  public Command goToPositionCommand(double targetPostion) {
    return Commands.defer(
            () ->
                startRun(
                    () -> {
                      inputs.finalSetpoint = targetPostion;
                      profiledPIDController.setGoal(targetPostion);
                    },
                    () -> {
                      profiledPIDController.calculate(inputs.elevatorPos);
                      State target = profiledPIDController.getSetpoint();
                      double acceleration =
                          (profiledPIDController.getSetpoint().velocity - lastSpeed)
                              / (Timer.getFPGATimestamp() - lastTime);
                      inputs.veolocitySetpoint = target.velocity;
                      inputs.motionSetpoint = target.position;
                      inputs.finalSetpoint = profiledPIDController.getGoal().position;
                      positionControl(
                          target.position, feedforward.calculate(target.velocity, acceleration));
                      lastSpeed = target.velocity;
                      lastTime = Timer.getFPGATimestamp();
                    }),
            Set.of(this))
        .until(atSetpoint)
        .withName("Elevator to " + targetPostion + " m high")
        .andThen(this::holdPosition);
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
}
