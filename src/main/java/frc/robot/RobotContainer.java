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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SingleColorFade;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSparkMax;
import frc.robot.subsystems.superstructure.ElevatorIO;
import frc.robot.subsystems.superstructure.ElevatorIOSim;
import frc.robot.subsystems.superstructure.ElevatorIOSparkMax;
import frc.robot.subsystems.superstructure.OutakeRollersIO;
import frc.robot.subsystems.superstructure.OutakeRollersIOTallonFX;
import frc.robot.subsystems.superstructure.SuperstructureController;
import frc.robot.subsystems.superstructure.WristIO;
import frc.robot.subsystems.superstructure.WristIOSim;
import frc.robot.subsystems.superstructure.WristTalonFXIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.Logger;
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
  private final Hopper hopper;
  private final SuperstructureController superstructure;
  private final Climber climber;
  private final Lights lights;

  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController auxController = new CommandXboxController(1);

  private final Trigger delayCondition;
  private final Trigger autoScore;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  ;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
        superstructure =
            new SuperstructureController(
                new ElevatorIOSparkMax(), new WristTalonFXIO(), new OutakeRollersIOTallonFX());
        climber = new Climber(new ClimberIOSparkMax());
        hopper = new Hopper(new HopperIOSparkMax());
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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3, drive::getPose));
        superstructure =
            new SuperstructureController(
                new ElevatorIOSim(), new WristIOSim(), new OutakeRollersIO() {});
        climber = new Climber(new ClimberIO() {});
        hopper = new Hopper(new HopperIOSparkMax());
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
        superstructure =
            new SuperstructureController(
                new ElevatorIO() {}, new WristIO() {}, new OutakeRollersIO() {});
        climber = new Climber(new ClimberIO() {});
        hopper = new Hopper(new HopperIO() {});
        break;
    }

    lights = new Lights();

    delayCondition = new Trigger(drive.autoElevator.or(auxController.y()));
    autoScore = new Trigger(drive.readyAutoScore.or(controller.rightBumper()));

    configureNamedCommands();
    configureTunerPaths();
    // Configure the button bindings
    configureButtonBindings();
    configureLEDbindings();

    RobotModeTriggers.disabled()
        .onFalse(Commands.parallel(superstructure.idle(), hopper.idleCommand()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

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

    // reset heading to when X button is pressed
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.leftBumper().onTrue(superstructure.intakeCoral().alongWith(hopper.intakeCommand()));

    /*
     * For all score commands: they do not do anything until auto score OR pressing Aux-Y
     * For all score commands: score is done automaticaly when auto score OR Driver-Rb
     * For all score commands: drives back and drops elevator automaticaly or waits until Aux-Y is released
     *
     * Aux-Lt without Aux-B: Move to L1
     * Aux-Rt without Aux-B: Move to L2
     * Aux-Lb without Aux-B: Move to L3
     * Aux-Lr without Aux-B: Move to L4
     *
     * Aux-Lt with Aux-B: Move to ground intake
     * Aux-Rt with Aux-B: Move to algae L2
     * Aux-Lb with Aux-B: Move to algae L3
     * Aux-Rb with Aux-B: Move to barge
     *
     */
    auxController
        .leftTrigger()
        .and(auxController.b().negate())
        .whileTrue(
            superstructure
                .scoreL1(() -> true, controller.rightBumper())
                .andThen(
                    superstructure
                        .holdHigh()
                        .until(drive.safeElevatorDown.and(auxController.y().negate()))));
    auxController
        .rightTrigger()
        .and(auxController.b().negate())
        .whileTrue(
            superstructure
                .scoreL2(delayCondition, autoScore)
                .andThen(
                    superstructure
                        .holdHigh()
                        .until(drive.safeElevatorDown.and(auxController.y().negate()))));
    auxController
        .leftBumper()
        .and(auxController.b().negate())
        .whileTrue(
            superstructure
                .scoreL3(delayCondition, autoScore)
                .andThen(
                    superstructure
                        .holdHigh()
                        .until(drive.safeElevatorDown.and(auxController.y().negate()))));
    auxController
        .rightBumper()
        .and(auxController.b().negate())
        .whileTrue(
            superstructure
                .scoreL4(delayCondition, autoScore)
                .andThen(
                    superstructure
                        .holdHigh()
                        .until(drive.safeElevatorDown.and(auxController.y().negate()))));
    auxController
        .leftTrigger()
        .and(auxController.b())
        .whileTrue(superstructure.intakeAlgaeGround());
    auxController.rightTrigger().and(auxController.b()).whileTrue(superstructure.intakeAlgaeL2());
    auxController.leftBumper().and(auxController.b()).whileTrue(superstructure.intakeAlgaeL3());
    auxController
        .rightBumper()
        .and(auxController.b())
        .whileTrue(superstructure.scoreBarge(() -> true, controller.b()));
  }

  private void configureLEDbindings() {}

  public void logSubsystems() {
    SmartDashboard.putData("drive", drive);
    SmartDashboard.putData("superstructure", superstructure);
    SmartDashboard.putData("Lights", lights);
  }

  public void updateMechanism2ds() {
    Pose3d[] mechanismPoses = {
      new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
      // new Pose3d(0, 0, elevator.get1stStageHeight(), new Rotation3d(0, 0, 0)),
      // new Pose3d(0, 0, elevator.get2ndStageHeight(), new Rotation3d(0, 0, 0)),
      // new Pose3d(0, 0, elevator.getCarrageHeight(), new Rotation3d(0, 0, 0))
    };
    Logger.recordOutput("mechanismPoses", mechanismPoses);
  }

  public void configureNamedCommands() {
    // NamedCommands.registerCommand("L4", mechanismControl.setState(State.levelFour).asProxy());
    // NamedCommands.registerCommand("L3", mechanismControl.setState(State.levelThree).asProxy());
    // NamedCommands.registerCommand(
    //     "bargeScore",
    //     Commands.runEnd(
    //             () -> {
    //               wrist.reqestIntakeVoltage(12);
    //             },
    //             () -> wrist.stopIntake())
    //         .withTimeout(0.5)
    //         .asProxy());
    // NamedCommands.registerCommand(
    //     "score",
    //     Commands.runEnd(
    //             () -> {
    //               wrist.reqestIntakeVoltage(3);
    //             },
    //             () -> wrist.stopIntake())
    //         .withTimeout(0.5)
    //         .asProxy());
    // NamedCommands.registerCommand(
    //     "waitUntilInake", Commands.waitUntil(wrist.detectsForward).asProxy());
    // NamedCommands.registerCommand(
    //     "algaeL3", mechanismControl.setState(State.algeaPickupL3).asProxy());
    // NamedCommands.registerCommand(
    //     "algaeL2", mechanismControl.setState(State.algaePickupL2).asProxy());
    // NamedCommands.registerCommand("store", mechanismControl.setState(State.store).asProxy());
    // NamedCommands.registerCommand("intake",
    // mechanismControl.setState(State.coralPickup).asProxy());
    // NamedCommands.registerCommand(
    //     "waitUntilSetpoint",
    //     Commands.run(() -> {}).until(mechanismControl.atDualSetPoint).asProxy());

    // NamedCommands.registerCommand("waitAlgea",
    // Commands.waitUntil(wrist.intakeStalled).asProxy());
    // NamedCommands.registerCommand(
    //     "algaeStore", mechanismControl.setState(State.algeaStore).asProxy());
    // NamedCommands.registerCommand("barge", mechanismControl.setState(State.barge).asProxy());
  }

  private void configureTunerPaths() {
    // Set up auto routines

    if (Constants.usePIDtuning) {
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
    }
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
