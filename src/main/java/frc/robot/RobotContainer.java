// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterRPMCommand;
import frc.robot.commands.TurretRPMCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.composite.CompositeIntakeSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOKraken;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOKraken;
import frc.robot.subsystems.turret.TurretIOSim;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandGenericHID operatorKeyboard =
      new CommandGenericHID(Constants.OPERATOR_KEYBOARD_PORT);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Shooter shooter =
      new Shooter(RobotBase.isReal() ? new ShooterIOKraken(15) : new ShooterIOSim(15));
  private final Turret turret =
      new Turret(RobotBase.isReal() ? new TurretIOKraken(14) : new TurretIOSim(14));
  private final Intake intake =
      new Intake(
          RobotBase.isReal()
              ? new IntakeIOKraken(
                  IntakeConstants.LIFT_CAN_ID,
                  IntakeConstants.ROLLER_CAN_ID,
                  IntakeConstants.CONVEYOR_CAN_ID,
                  Constants.CANIVORE_BUS)
              : new IntakeIOSim());
  private final Indexer indexer =
      new Indexer(
          RobotBase.isReal()
              ? new IndexerIOKraken(
                  IndexerConstants.LEADER_CAN_ID,
                  IndexerConstants.FOLLOWER_CAN_ID,
                  Constants.CANIVORE_BUS)
              : new IndexerIOSim());
  private final CompositeIntakeSubsystem compositeIntake =
      new CompositeIntakeSubsystem(intake, indexer);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
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
        break;
    }

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
    configureOperatorBindings();
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

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    Trigger shooterForwardOnly = controller.rightBumper().and(controller.leftBumper().negate());
    Trigger shooterReverseOnly = controller.leftBumper().and(controller.rightBumper().negate());

    shooterForwardOnly.whileTrue(new ShooterRPMCommand(shooter, true));
    shooterReverseOnly.whileTrue(new ShooterRPMCommand(shooter, false));

    Trigger turretRightOnly = controller.povRight().and(controller.povLeft().negate());
    Trigger turretLeftOnly = controller.povLeft().and(controller.povRight().negate());

    turretRightOnly.whileTrue(new TurretRPMCommand(turret, true));
    turretLeftOnly.whileTrue(new TurretRPMCommand(turret, false));
  }

  private void configureOperatorBindings() {

    // ── Key 1 : Composite forward ─────────────────────────────────────────
    // Held: lowers arm → waits for arm → runs rollers + indexer forward
    // Released: stops rollers and indexer
    // operatorKeyboard.button(1).whileTrue(compositeIntake.compositeForwardCommand());
    controller.povUp().whileTrue(compositeIntake.compositeForwardCommand());

    // ── Key 2 : Composite reverse ─────────────────────────────────────────
    // Held: runs rollers + indexer in reverse (eject / un-jam)
    // Released: stops both
    // operatorKeyboard.button(2).whileTrue(compositeIntake.compositeReverseCommand());
    controller.povDown().whileTrue(compositeIntake.compositeReverseCommand());

    // ── Key 3 : Intake lower ──────────────────────────────────────────────
    // Held: commands arm to down position (MotionMagic holds it)
    // Released: arm holds position (no explicit cancel)
    // operatorKeyboard.button(3).onTrue(compositeIntake.intakeLowerCommand());
    controller.leftTrigger().whileTrue(compositeIntake.intakeRaiseCommand());

    // ── Key 4 : Intake raise ──────────────────────────────────────────────
    // Held: commands arm to up/stow position
    // operatorKeyboard.button(4).onTrue(compositeIntake.intakeRaiseCommand());
    controller.rightTrigger().whileTrue(compositeIntake.intakeLowerCommand());

    // ── Key 5 : Intake forward ────────────────────────────────────────────
    // Held: runs rollers + conveyor forward
    // Released: stops rollers
    operatorKeyboard.button(5).whileTrue(compositeIntake.intakeForwardCommand());

    // ── Key 6 : Intake reverse ────────────────────────────────────────────
    // Held: runs rollers + conveyor in reverse
    // Released: stops rollers
    operatorKeyboard.button(6).whileTrue(compositeIntake.intakeReverseCommand());

    // ── Key 7 : Indexer forward ───────────────────────────────────────────
    // Held: runs indexer forward
    // Released: stops indexer
    operatorKeyboard.button(7).whileTrue(compositeIntake.indexerForwardCommand());

    // ── Key 8 : Indexer reverse ───────────────────────────────────────────
    // Held: runs indexer in reverse
    // Released: stops indexer
    operatorKeyboard.button(8).whileTrue(compositeIntake.indexerReverseCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void stopMechanisms() {
    shooter.stop();
    turret.stop();
    drive.stop();
  }
}
