// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

// import static frc.robot.subsystems.Vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SingleColorFade;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mechanisms.MechanismControl;
import frc.robot.subsystems.mechanisms.MechanismControl.State;
import frc.robot.subsystems.mechanisms.climber.Climber;
import frc.robot.subsystems.mechanisms.climber.ClimberIO;
import frc.robot.subsystems.mechanisms.climber.ClimberIOSparkMax;
import frc.robot.subsystems.mechanisms.elevator.Elevator;
import frc.robot.subsystems.mechanisms.elevator.ElevatorIO;
import frc.robot.subsystems.mechanisms.elevator.ElevatorIOSim;
import frc.robot.subsystems.mechanisms.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.mechanisms.wrist.Wrist;
import frc.robot.subsystems.mechanisms.wrist.WristIO;
import frc.robot.subsystems.mechanisms.wrist.WristIOSim;
import frc.robot.subsystems.mechanisms.wrist.WristIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.Binding;
import frc.robot.util.ScorePositionSelector;

import java.util.EnumMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Climber climber;
  private final Lights lights;

  private final MechanismControl mechanismControl;

  private final Vision vision;

  private final ScorePositionSelector selector;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController auxController = new CommandXboxController(1);

  private Trigger anyPov;

  private LoggedMechanism2d superstructure2d =
      new LoggedMechanism2d(Units.inchesToMeters(2), Units.inchesToMeters(32.5 * 4));
  private LoggedMechanismRoot2d superstructureRoot = superstructure2d.getRoot("elevatorBase", 0, 0);
  private LoggedMechanismLigament2d elevatorVisual =
      superstructureRoot.append(
          new LoggedMechanismLigament2d("elevator", Units.inchesToMeters(9), 90));

  private enum AutoEnums {
    coral,
    algea,
    intake
  }

  private EnumMap<AutoEnums, Command> leftCommandMap = new EnumMap<>(AutoEnums.class);
  private EnumMap<AutoEnums, Command> rightCommandMap = new EnumMap<>(AutoEnums.class);

  private Supplier<AutoEnums> autoEnumSupplier;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    lights = new Lights();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement, drive::getPose, new VisionIO() {}, new VisionIO()
        // {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2),
                new VisionIOPhotonVision(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3));
        elevator = new Elevator(new ElevatorIOSparkMax());
        wrist = new Wrist(new WristIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision = new Vision(drive::addVisionMeasurement, drive::getPose);

        // drive::addVisionMeasurement,
        // drive::getPose,
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
        // new VisionIOPhotonVisionSim(
        //     VisionConstants.camera3Name, VisionConstants.robotToCamera3, drive::getPose));
        elevator = new Elevator(new ElevatorIOSim());
        wrist = new Wrist(new WristIOSim());
        climber = new Climber(new ClimberIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement, drive::getPose, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    mechanismControl = new MechanismControl(drive, elevator, wrist, climber, lights);

    configureNamedCommands();

    selector = new ScorePositionSelector(mechanismControl.setState(State.store).withName("Store"));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureLEDbindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // RobotModeTriggers.disabled()
    //     .onTrue(
    //         mechanismControl
    //             .setState(State.idle)
    //             .ignoringDisable(true)
    //             .alongWith(Commands.runOnce(() -> drive.configCoastMode())))
    //     .onFalse(Commands.runOnce(() -> drive.configBreakMode()));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    lights.setDefaultCommand(
        (new SingleColorFade(new Color(80, 7, 120), lights)
                .andThen(new SingleColorFade(new Color(255, 209, 0), lights)))
            .repeatedly());

    // Lock to 0 when A button is held
    auxController.a().onTrue(Commands.runOnce(() -> drive.setWheelsAndCoast()));
    controller
        .a()
        .onTrue(mechanismControl.setState(State.levelFour).withName("L4"))
        .onFalse(mechanismControl.setState(State.store));
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveRobot(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> -controller.getRightX()));

    // Switch to X pattern when X button is pressed
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Reset gyro to 0 when B button is pressed
    controller
        .y()
        .whileTrue(
            DriveCommands.pointTowardsReefCommand(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    autoEnumSupplier = () -> {
        if(wrist.intakeStalled.getAsBoolean()) {
            return AutoEnums.algea;
        } else if (wrist.detectsForward.getAsBoolean()) {
            return AutoEnums.coral;
        } else {
            return AutoEnums.intake;
        }
    };

    leftCommandMap.put(AutoEnums.coral, drive.getLeftCoralDriveCommand(mechanismControl.elevatorUp));
    leftCommandMap.put(AutoEnums.algea, new InstantCommand());
    leftCommandMap.put(AutoEnums.intake, drive.getLeftSourceDriveCommand());

    rightCommandMap.put(AutoEnums.coral, drive.getRightCoralDriveCommand(mechanismControl.elevatorUp));
    rightCommandMap.put(AutoEnums.algea, drive.getSourceDriveCommand());
    rightCommandMap.put(AutoEnums.intake, drive.getRightSourceDriveCommand());
    
    // controller
    //     .leftTrigger()
    //     .whileTrue(
    //         new ConditionalCommand(
    //             new InstantCommand(),
    //             drive.getLeftCoralDriveCommand(mechanismControl.elevatorUp),
    //             wrist.intakeStalled));

    controller.leftTrigger().whileTrue(Commands.select(leftCommandMap, autoEnumSupplier));

    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         new ConditionalCommand(
    //             drive.getProccesorDriveCommand(),
    //             drive.getRightCoralDriveCommand(mechanismControl.elevatorUp),
    //             wrist.intakeStalled));

    controller.rightTrigger().whileTrue(Commands.select(rightCommandMap, autoEnumSupplier));

    anyPov = new Trigger(() -> controller.getHID().getPOV() != -1);
    anyPov.onTrue(mechanismControl.setState(State.climb));

    controller
        .rightBumper()
        .whileTrue(
            new ConditionalCommand(
                Commands.run(() -> wrist.reqestIntakeVoltage(6)),
                Commands.run(() -> wrist.reqestIntakeVoltage(12)),
                wrist.intakeStalled))
        .onFalse(Commands.runOnce(() -> wrist.reqestIntakeVoltage(0)));
    controller.leftBumper().onTrue(mechanismControl.setState(State.coralPickup));

    //
    selector
        .addBinding(
            new Binding(
                auxController.leftTrigger().and(auxController.b().negate()),
                mechanismControl.setState(State.levelOne).withName("L1")))
        .addBinding(
            new Binding(
                auxController.rightTrigger().and(auxController.b().negate()),
                mechanismControl.setState(State.levelTwo).withName("L2")))
        .addBinding(
            new Binding(
                auxController.leftBumper().and(auxController.b().negate()),
                mechanismControl.setState(State.levelThree).withName("L3")))
        .addBinding(
            new Binding(
                auxController.rightBumper().and(auxController.b().negate()),
                mechanismControl.setState(State.levelFour).withName("L4")))
        .addBinding(
            new Binding(
                (auxController.leftTrigger().or(auxController.rightTrigger()))
                    .and(auxController.b()),
                mechanismControl.setState(State.algaePickupL2).withName("AlgeaL2")))
        .addBinding(
            new Binding(
                (auxController.leftBumper().or(auxController.rightBumper())).and(auxController.b()),
                mechanismControl.setState(State.algeaPickupL3).withName("AlgeaL3")));

    auxController
        .axisMagnitudeGreaterThan(1, 0.1)
        .whileTrue(
            elevator
                .manualRunCommand(() -> auxController.getLeftY())
                .alongWith(mechanismControl.setState(State.idle).repeatedly()))
        .whileFalse(elevator.holdPosition());

    auxController
        .axisMagnitudeGreaterThan(5, 0.1)
        .whileTrue(
            climber
                .voltageCommand(() -> auxController.getRightY() * 12.0)
                .alongWith(mechanismControl.setState(State.idle).repeatedly()))
        .onFalse(new InstantCommand(climber::stop));

    auxController.povRight().onTrue(mechanismControl.setState(State.coralPickup));
    auxController
        .povLeft()
        .onTrue(mechanismControl.setState(State.reverse))
        .onFalse(mechanismControl.setState(State.idle));

    auxController.povUp().onTrue(mechanismControl.setState(State.climberPrep));
    auxController.a().whileTrue(elevator.sysIdRoutine());

    auxController.povDown().onTrue(mechanismControl.setState(State.climb));

    auxController
        .y()
        .onTrue(mechanismControl.setState(State.algeaGround))
        .onFalse(mechanismControl.setState(State.algeaStore));
  }

  private void configureLEDbindings() {
    mechanismControl
        .atDualSetPoint
        .onTrue(lights.solidAnimation(new Color(0, 255, 0), "atDualSetPoint"))
        .onFalse(lights.solidAnimation(new Color(255, 0, 0), "NOT atDualSetPoint"));
    wrist.detectsBoth.onTrue(lights.strobeAnimation(new Color(0, 255, 0), "detects both"));
    wrist.intakeStalled.onTrue(lights.strobeAnimation(new Color(0, 0, 255), "intake Stalled"));
    drive.autoElevator.onFalse(lights.solidAnimation(new Color(0, 0, 255), "NOT autoElevator"));
  }

  public void logSubsystems() {
    SmartDashboard.putData("drive", drive);
    SmartDashboard.putData("elevator", elevator);
    SmartDashboard.putData("wrist", wrist);
    SmartDashboard.putData("Lights", lights);
    selector.log();
  }

  public void updateMechanism2ds() {
    elevatorVisual.setLength(Units.inchesToMeters(9.0) + elevator.getCarrageHeight());
    Logger.recordOutput("mech2d/superstructure", superstructure2d);
    Pose3d[] mechanismPoses = {
      new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
      new Pose3d(0, 0, elevator.get1stStageHeight(), new Rotation3d(0, 0, 0)),
      new Pose3d(0, 0, elevator.get2ndStageHeight(), new Rotation3d(0, 0, 0)),
      new Pose3d(0, 0, elevator.getCarrageHeight(), new Rotation3d(0, 0, 0))
    };
    Logger.recordOutput("mechanismPoses", mechanismPoses);
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("L4", mechanismControl.setState(State.levelFour).asProxy());
    NamedCommands.registerCommand(
        "score",
        Commands.runEnd(
                () -> {
                  wrist.reqestIntakeVoltage(12);
                },
                () -> wrist.stopIntake())
            .withTimeout(0.5)
            .asProxy());
    NamedCommands.registerCommand(
        "waitUntilInake", Commands.waitUntil(wrist.detectsForward).asProxy());
    NamedCommands.registerCommand(
        "algaeL3", mechanismControl.setState(State.algeaPickupL3).asProxy());
    NamedCommands.registerCommand(
        "algaeL2", mechanismControl.setState(State.algaePickupL2).asProxy());
    NamedCommands.registerCommand("store", mechanismControl.setState(State.store).asProxy());
    NamedCommands.registerCommand("intake", mechanismControl.setState(State.coralPickup).asProxy());
    NamedCommands.registerCommand(
        "waitUntilSetpoint",
        Commands.run(() -> {}).until(mechanismControl.atDualSetPoint).asProxy());

    NamedCommands.registerCommand("waitAlgea", Commands.waitUntil(wrist.intakeStalled).asProxy());
    NamedCommands.registerCommand(
        "algaeStore", mechanismControl.setState(State.algeaStore).asProxy());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
