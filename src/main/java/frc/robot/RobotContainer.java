// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOKraken;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.*;
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
  private final Turret turret;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Intake intake =
      new Intake(RobotBase.isReal() ? new IntakeIOKraken() : new IntakeIOSim());
\   private final Indexer indexer =
      new Indexer(RobotBase.isReal() ? new IndexerIOKraken() : new IndexerIOSim());

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
        turret = new Turret(new TurretIOKraken(), drive::getPose, drive::getChassisSpeeds);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.leftCameraName, VisionConstants.robotToLeftCamera),
                new VisionIOPhotonVision(
                    VisionConstants.rightCameraName, VisionConstants.robotToRightCamera));
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
        turret = new Turret(new TurretIOSim(), drive::getPose, drive::getChassisSpeeds);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.leftCameraName,
                    VisionConstants.robotToLeftCamera,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.rightCameraName,
                    VisionConstants.robotToRightCamera,
                    drive::getPose));
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
        turret = new Turret(new TurretIO() {}, drive::getPose, drive::getChassisSpeeds);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    NamedCommands.registerCommand(
        "Shoot",
        new ParallelCommandGroup(
            turret.setGoal(Turret.TurretGoal.SCORING), indexer.setState(Indexer.State.SCORING)));
    NamedCommands.registerCommand(
        "Stop Shooting",
        new ParallelCommandGroup(
            turret.setGoal(Turret.TurretGoal.IDLE), indexer.setState(Indexer.State.IDLE)));
    NamedCommands.registerCommand(
        "Intake down", intake.setState(Intake.PivotState.DOWN, Intake.RollerState.IDLE));

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
    driveBindings();
    configureBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void driveBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  private void configureBindings() {
    controller
        .rightTrigger()
        .onTrue(
            new ParallelCommandGroup(
                turret.setGoal(Turret.TurretGoal.SCORING),
                indexer.setState(Indexer.State.SCORING)));
    controller
        .rightTrigger()
        .onFalse(
            new ParallelCommandGroup(
                turret.setGoal(Turret.TurretGoal.IDLE), indexer.setState(Indexer.State.IDLE)));

    controller
        .rightBumper()
        .onTrue(
            new ParallelCommandGroup(
                turret.setGoal(Turret.TurretGoal.PASSING),
                indexer.setState(Indexer.State.SCORING)));
    controller
        .rightBumper()
        .onFalse(
            new ParallelCommandGroup(
                turret.setGoal(Turret.TurretGoal.IDLE), indexer.setState(Indexer.State.IDLE)));

    controller
        .leftTrigger()
        .onTrue(intake.setState(Intake.PivotState.IDLE, Intake.RollerState.INTAKING));
    controller
        .leftTrigger()
        .onFalse(intake.setState(Intake.PivotState.IDLE, Intake.RollerState.IDLE));

    controller.povDown().onTrue(intake.setState(Intake.PivotState.DOWN, Intake.RollerState.IDLE));
    controller.povDown().onFalse(intake.setState(Intake.PivotState.IDLE, Intake.RollerState.IDLE));

    controller.povUp().onTrue(intake.setState(Intake.PivotState.UP, Intake.RollerState.IDLE));

    controller
        .povRight()
        .onTrue(
            new ParallelCommandGroup(
                turret.setGoal(Turret.TurretGoal.TUNING), indexer.setState(Indexer.State.SCORING)));
    controller
        .povRight()
        .onFalse(
            new ParallelCommandGroup(
                turret.setGoal(Turret.TurretGoal.IDLE), indexer.setState(Indexer.State.IDLE)));

    //    controller.rightBumper().whileTrue(compositeIntake.loadShooter());
    //    controller.leftTrigger().whileTrue(compositeIntake.compositeForwardCommandandPivot());
    //    controller.b().onTrue(compositeIntake.intakeRaiseCommand());
    //     controller.rightBumper().onTrue(turret.setGoal(Turret.TurretGoal.TUNING));
    //     controller.rightBumper().onFalse(turret.setGoal(Turret.TurretGoal.OFF));

    //    operator.leftTrigger().whileTrue(compositeIntake.compositeReverseCommand());
    //    controller.povRight().whileTrue(compositeIntake.intakeZeroCommand());
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
    turret.setGoal(Turret.TurretGoal.OFF);
    intake.stop();
    indexer.stop();
    drive.stop();
  }
}
