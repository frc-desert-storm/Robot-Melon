// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.FieldConstants.HUB_CENTER;
import static frc.robot.Constants.TurretConstants.FLYWHEEL_RADIUS;
import static frc.robot.Constants.TurretConstants.ROBOT_TO_TURRET_TRANSFORM;
import static frc.robot.Constants.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOKraken;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOKraken;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.FuelSim;
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
  private final Turret turret;
  private final Vision vision;
  private final Intake intake =
      new Intake(RobotBase.isReal() ? new IntakeIOKraken() : new IntakeIOSim());
  private final Indexer indexer;

  private final Superstructure superstructure;

  public final FuelSim fuelSim = new FuelSim();

  private static final int SIM_FUEL_CAPACITY = 65;
  private static final double SIM_SHOT_INTERVAL_SEC = 0.18;
  private static final double SIM_MAX_INTAKE_FUEL_PER_SEC = 12.0;
  private int simFuelStored = 8;
  private double lastSimShotTime = 0.0;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        turret =
            new Turret(new TurretIOKraken(), drive::getPose, this::getFieldRelativeChassisSpeeds);
        indexer = new Indexer(new IndexerIOKraken(), turret::getDistanceToTarget);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.leftCameraName, VisionConstants.robotToLeftCamera),
                new VisionIOPhotonVision(
                    VisionConstants.rightCameraName, VisionConstants.robotToRightCamera),
                new VisionIOPhotonVision(
                    VisionConstants.turretCameraName,
                    timestamp -> getRobotToTurretCamera(turret.getTurnPositionAt(timestamp))));
        superstructure =
            new Superstructure(
                turret, indexer, drive::getPose, this::getFieldRelativeChassisSpeeds);
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
        turret = new Turret(new TurretIOSim(), drive::getPose, this::getFieldRelativeChassisSpeeds);
        indexer = new Indexer(new IndexerIOSim(), turret::getDistanceToTarget);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.leftCameraName, robotToLeftCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.rightCameraName, robotToRightCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.turretCameraName,
                    () -> getRobotToTurretCamera(turret.getTurnPosition()),
                    drive::getPose));
        superstructure =
            new Superstructure(
                turret, indexer, drive::getPose, this::getFieldRelativeChassisSpeeds);
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
        turret = new Turret(new TurretIO() {}, drive::getPose, this::getFieldRelativeChassisSpeeds);
        indexer = new Indexer(new IndexerIO() {}, turret::getDistanceToTarget);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        superstructure =
            new Superstructure(
                turret, indexer, drive::getPose, this::getFieldRelativeChassisSpeeds);
        break;
    }

    registerNamedCommands();

    configureFuelSim();

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

    Logger.recordOutput("left", robotToLeftCamera);
    Logger.recordOutput("right", robotToRightCamera);
    Logger.recordOutput("turret", getRobotToTurretCamera(Rotation2d.kZero));
    Logger.recordOutput("line", new Pose2d(HUB_CENTER.in(Meters), 0.0, new Rotation2d()));
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
  private static final double SHOOTING_DRIVE_SCALE = 0.3;

  private double driveSpeedScale() {
    var state = superstructure.getState();
    boolean shooting =
        state == Superstructure.SuperstructureState.WINDUP
            || state == Superstructure.SuperstructureState.SHOOTING
            || state == Superstructure.SuperstructureState.TESTING
            || state == Superstructure.SuperstructureState.TESTING_WINDUP;
    return shooting ? SHOOTING_DRIVE_SCALE : 1.0;
  }

  private void driveBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            this::driveSpeedScale));

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
    controller.rightTrigger().whileTrue(superstructure.shoot());

    controller.rightBumper().whileTrue(superstructure.test());

    controller.leftTrigger().whileTrue(intake.intake());

    controller.povUp().onTrue(intake.zeroExtension());

    controller.leftBumper().whileTrue(intake.retract());

    //    controller
    //        .leftBumper()
    //        .whileTrue(
    //            Commands.runOnce(
    //                () -> {
    //                  intake.setState(ExtensionState.UP, RollerState.INTAKING);
    //                }));

    //    controller.povRight().whileTrue(superstructure.reverse());
  }

  private void configureFuelSim() {
    fuelSim.setSubticks(10);
    fuelSim.setLoggingFrequency(50);
    fuelSim.spawnStartingFuel();

    fuelSim.registerRobot(
        Dimensions.FULL_WIDTH,
        Dimensions.FULL_LENGTH,
        Dimensions.BUMPER_HEIGHT,
        drive::getPose,
        this::getFieldRelativeChassisSpeeds);

    double halfLengthM = Dimensions.FULL_LENGTH.in(Meters) / 2.0;
    double intakeDepthM = Inches.of(12).in(Meters);
    double halfIntakeWidthM = Inches.of(14).in(Meters);
    fuelSim.registerIntake(
        -halfLengthM - intakeDepthM,
        -halfLengthM,
        -halfIntakeWidthM,
        halfIntakeWidthM,
        () -> intake.isExtended() && intake.isIntaking() && simFuelStored < SIM_FUEL_CAPACITY,
        () -> {
          if (simFuelStored < SIM_FUEL_CAPACITY) {
            simFuelStored++;
          }
        },
        SIM_MAX_INTAKE_FUEL_PER_SEC);

    fuelSim.start();
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  fuelSim.clearFuel();
                  fuelSim.spawnStartingFuel();
                  simFuelStored = 0;
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }

  public void updateFuelSimShooting() {
    Logger.recordOutput("FuelSim/StoredFuel", simFuelStored);
    var state = superstructure.getState();
    boolean shooting =
        state == Superstructure.SuperstructureState.SHOOTING
            || state == Superstructure.SuperstructureState.TESTING;
    if (!shooting || simFuelStored <= 0) {
      return;
    }
    if (!turret.ready()) {
      return;
    }
    if (turret.getFlywheelSpeed().abs(RPM) < 100) {
      return;
    }
    double now = Timer.getFPGATimestamp();
    if (now - lastSimShotTime < SIM_SHOT_INTERVAL_SEC) {
      return;
    }
    lastSimShotTime = now;
    simFuelStored--;

    fuelSim.launchFuel(
        TurretCalculator.angularToLinearVelocity(turret.getFlywheelSpeed(), FLYWHEEL_RADIUS),
        Degrees.of(90).minus(turret.getHoodPosition()),
        Radians.of(turret.getTurnPosition().getRadians()),
        ROBOT_TO_TURRET_TRANSFORM.getMeasureZ());
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "Intake out",
        Commands.runOnce(
            () -> intake.setState(Intake.ExtensionState.EXTENDING, Intake.RollerState.IDLE),
            intake));
    NamedCommands.registerCommand(
        "Intake in",
        Commands.runOnce(
            () -> intake.setState(Intake.ExtensionState.RETRACTING, Intake.RollerState.INTAKING),
            intake));
    NamedCommands.registerCommand(
        "Start intaking",
        Commands.runOnce(
            () -> intake.setState(Intake.ExtensionState.EXTENDING, Intake.RollerState.INTAKING),
            intake));
    NamedCommands.registerCommand(
        "Stop intaking",
        Commands.runOnce(
            () -> intake.setState(Intake.ExtensionState.IDLE, Intake.RollerState.IDLE), intake));
    NamedCommands.registerCommand(
        "Zero intake", intake.zeroExtension().withTimeout(1.0).repeatedly().withTimeout(3.0));
    NamedCommands.registerCommand(
        "Start shooting",
        Commands.sequence(
            Commands.runOnce(
                () -> superstructure.applyState(Superstructure.SuperstructureState.WINDUP))));
    NamedCommands.registerCommand(
        "Stop shooting",
        Commands.runOnce(
            () -> superstructure.applyState(Superstructure.SuperstructureState.IDLE),
            superstructure));
  }

  private ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void stopMechanisms() {
    CommandScheduler.getInstance().schedule(superstructure.idle());
    drive.stop();
  }
}
