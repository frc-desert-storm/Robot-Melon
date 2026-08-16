// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.turret.TurretCalculator;
import java.io.IOException;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class Dimensions {
    public static final Distance BUMPER_THICKNESS = Inches.of(3.5); // frame to edge of bumper
    public static final Distance BUMPER_HEIGHT = Inches.of(5); // height from floor to top of bumper
    public static final Distance FRAME_SIZE_Y = Inches.of(28); // left to right (y-axis)
    public static final Distance FRAME_SIZE_X = Inches.of(26); // front to back (x-axis)

    public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
    public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
  }

  public static final class IntakeConstants {
    private IntakeConstants() {}
    /** Gear ratio between extension motor output shaft and the mechanism. */
    public static final double EXTENSION_GEAR_RATIO =
        (64.0 / 14) * (26.0 / 15); // 14:64 gear, 15:26 belt

    /** Gear ratio between roller motor output shaft and the roller mechanism. */
    public static final double ROLLER_GEAR_RATIO = (25.0 / 12); // 12:25 belt

    public static final int EXTENSION_FOLLOWER_CAN_ID = 44;
    public static final int EXTENSION_CAN_ID = 43;
    public static final int ROLLER_CAN_ID = 50;

    public static final Distance STOW_POSE = Inch.of(3.5);
    public static final Distance INTAKING_POSE = Inch.of(11.52);

    // MotionMagic gains for extension (tune to robot)
    public static final Slot0Configs EXTENSION_GAINS =
        new Slot0Configs()
            .withKP(24)
            .withKI(0)
            .withKD(1.5)
            .withKS(0)
            .withKV(0.4)
            .withKA(0)
            .withKG(0);

    public static final double EXTENSION_CRUISE_RPS = 160.0; // motor rotations per second
    public static final double EXTENSION_ACCEL_RPS2 = 360.0; // motor rotations per second²
    public static final double EXTENSION_JERK_RPS3 = 1600.0;
    public static final MotionMagicConfigs EXTENSION_MOTION_MAGIC =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(EXTENSION_CRUISE_RPS)
            .withMotionMagicAcceleration(EXTENSION_ACCEL_RPS2)
            .withMotionMagicJerk(EXTENSION_JERK_RPS3);
    public static final CurrentLimitsConfigs EXTENSION_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(20);
    public static final MotorOutputConfigs EXTENSION_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    public static final Slot0Configs EXTENSION_LEFT_GAINS =
        new Slot0Configs()
            .withKP(24)
            .withKI(0)
            .withKD(1.5)
            .withKS(0)
            .withKV(0.4)
            .withKA(0)
            .withKG(0);

    public static final double EXTENSION_LEFT_CRUISE_RPS = 160.0; // motor rotations per second
    public static final double EXTENSION_LEFT_ACCEL_RPS2 = 360.0; // motor rotations per second²
    public static final double EXTENSION_LEFT_JERK_RPS3 = 1600.0;
    public static final MotionMagicConfigs EXTENSION_LEFT_MOTION_MAGIC =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(EXTENSION_LEFT_CRUISE_RPS)
            .withMotionMagicAcceleration(EXTENSION_LEFT_ACCEL_RPS2)
            .withMotionMagicJerk(EXTENSION_LEFT_JERK_RPS3);
    public static final CurrentLimitsConfigs EXTENSION_LEFT_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withStatorCurrentLimit(20);
    public static final MotorOutputConfigs EXTENSION_LEFT_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    // Roller gains, current limits, and output config
    public static final Slot0Configs ROLLER_GAINS = new Slot0Configs().withKP(0.2).withKV(1.0);
    public static final CurrentLimitsConfigs ROLLER_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withStatorCurrentLimit(60);
    public static final MotorOutputConfigs ROLLER_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
  }

  public static final class IndexerConstants {
    private IndexerConstants() {}

    public static final int INDEXER_ID = 63;

    public static final double INDEXER_GEAR_RATIO = (56.0 / 12) * 9; // 12:56 chain, max planetary

    public static final Slot0Configs INDEXER_GAINS = new Slot0Configs().withKP(8);

    public static final CurrentLimitsConfigs INDEXER_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(15).withStatorCurrentLimit(40);
  }

  public static class TurretConstants {
    public static final int TURN_ID = 63;
    public static final int HOOD_ID = 63;
    public static final int FLYWHEEL_ID = 63;
    public static final int FLYWHEEL_FOLLOWER_ID = 63;

    public static final double TURN_TO_TURRET_RATIO =
        (50.0 / 16) * (20.0 / 16) * (160.0 / 10); // 16:50 gear, 16:20 belt, 10:160 gear
    public static final double HOOD_MOTOR_RATIO =
        (54.0 / 10) * (160.0 / 10); // 10:54 gear, 10:160 gear
    public static final double FLYWHEEL_MOTOR_RATIO = 0.5; // 1:2

    public static final Slot0Configs TURN_GAINS =
        new Slot0Configs().withKP(400).withKD(0.1).withKS(2);

    public static final Slot0Configs HOOD_GAINS =
        new Slot0Configs().withKP(256).withKD(5).withKS(0.28);

    public static final Slot0Configs FLYWHEEL_GAINS =
        new Slot0Configs().withKP(10).withKD(1.0).withKS(10).withKV(0.3);

    public static final CurrentLimitsConfigs TURN_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(20);

    public static final CurrentLimitsConfigs HOOD_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(20);

    public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(60);

    public static final MotorOutputConfigs TURN_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final MotorOutputConfigs HOOD_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    public static final MotorOutputConfigs FLYWHEEL_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    public static final FeedbackConfigs FLYWHEEL_FEEDBACK_CONFIGS =
        new FeedbackConfigs()
            .withVelocityFilterTimeConstant(Seconds.of(0.01))
            .withRotorToSensorRatio(FLYWHEEL_MOTOR_RATIO);

    public static final Distance DISTANCE_ABOVE_FUNNEL =
        Inches.of(20); // how high to clear the funnel
    public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
        new Transform3d(
            new Translation3d(Inches.zero(), Inches.zero(), Inches.of(21.008250)),
            Rotation3d.kZero);
    public static final Distance FLYWHEEL_RADIUS = Inches.of(2);
    public static final int LOOKAHEAD_ITERATIONS = 3;

    public static final Angle MIN_TURN_ANGLE = Degrees.of(-240);
    public static final Angle MAX_TURN_ANGLE = Degrees.of(240);
    public static final Angle TURNAROUND_ZONE = Degrees.of(30);

    public static final Angle MIN_HOOD_ANGLE = Degrees.of(12);
    public static final Angle MAX_HOOD_ANGLE = Degrees.of(42);

    public static final Distance EXTRA_DUCK_DISTANCE = Meters.of(0.5);
    public static final Time DUCK_TIME = Seconds.of(0.4);

    public static final Current HOOD_STALL_CURRENT = Amps.of(10);
    public static final AngularVelocity HOOD_STALL_ANGULAR_VELOCITY = RadiansPerSecond.of(0.3);
    public static final Voltage HOOD_ZEROING_VOLTAGE = Volts.of(-1);

    public static final AngularVelocity FLYWHEEL_FUDGE_AMOUNT = RPM.of(10);

    public static final AngularVelocity FLYWHEEL_SCORING_OVERRIDE = RPM.of(2700);
    public static final Angle HOOD_SCORING_OVERRIDE = Degrees.of(25);

    public static final AngularVelocity FLYWHEEL_PASSING_OVERRIDE = RPM.of(2800);
    public static final Angle HOOD_PASSING_OVERRIDE = Degrees.of(27);

    public static final Translation3d PASSING_SPOT_LEFT =
        new Translation3d(
            Inches.of(90),
            Constants.FieldConstants.FIELD_WIDTH.div(2).plus(Inches.of(85)),
            Inches.zero());
    public static final Translation3d PASSING_SPOT_RIGHT =
        new Translation3d(
            Inches.of(90),
            Constants.FieldConstants.FIELD_WIDTH.div(2).minus(Inches.of(85)),
            Inches.zero());

    public static final InterpolatingTreeMap<Double, TurretCalculator.ShotData> SHOT_MAP =
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), TurretCalculator.ShotData::interpolate);

    public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

    static {
      SHOT_MAP.put(1.75, new TurretCalculator.ShotData(RPM.of(2600), Degrees.of(22)));
      TOF_MAP.put(1.75, 1.02);
      SHOT_MAP.put(2.0, new TurretCalculator.ShotData(RPM.of(2750), Degrees.of(22)));
      TOF_MAP.put(2.0, 1.05);
      SHOT_MAP.put(3.2, new TurretCalculator.ShotData(RPM.of(3100), Degrees.of(22)));
      TOF_MAP.put(3.2, 1.16);
      SHOT_MAP.put(4.0, new TurretCalculator.ShotData(RPM.of(3250), Degrees.of(24)));
      TOF_MAP.put(4.0, 1.25);
      SHOT_MAP.put(5.0, new TurretCalculator.ShotData(RPM.of(3550), Degrees.of(28)));
      TOF_MAP.put(5.0, 1.33);
      SHOT_MAP.put(6.0, new TurretCalculator.ShotData(RPM.of(4200), Degrees.of(28)));
      TOF_MAP.put(6.0, 1.40);
    }

    public static final Time ACTIVE_PRESHOOT_TIME = Seconds.of(2);
    public static final Time ACTIVE_POSTSHOOT_TIME = Seconds.of(1);

    public static final double SCORE_WINDUP_SECONDS = 0.25;
    public static final double PASS_WINDUP_SECONDS = 0.5;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);

    public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

    public static final Translation3d HUB_BLUE =
        new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Translation3d HUB_RED =
        new Translation3d(
            FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Distance FUNNEL_RADIUS = Inches.of(24);
    public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

    public static final Distance TRENCH_BUMP_X =
        Inches.of(181.56); // x position of the center of the trench and bump
    public static final Distance TRENCH_WIDTH = Inches.of(49.86); // y width of the trench
    public static final Distance TRENCH_BUMP_LENGTH =
        Inches.of(40); // x length of the trench and bump
    public static final Distance TRENCH_BAR_WIDTH = Inches.of(4); // x width of the trench bar
    public static final Distance TRENCH_BLOCK_WIDTH =
        Inches.of(12); // y width of block separating bump and trench
    public static final Distance BUMP_WIDTH = Inches.of(73); // y width of bump

    public static final Distance TRENCH_CENTER = Dimensions.FULL_LENGTH.div(2).plus(Inches.of(7));
    // public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);

    public static final Distance TOWER_X = Inches.of(49.25);
    public static final Distance TOWER_CENTER_Y = FIELD_WIDTH.div(2).minus(Inches.of(11.46));
    public static final Distance TOWER_CENTER_X = Inches.of(18);
    public static final Distance TOWER_WIDTH = Inches.of(51);
  }

  public static final class VisionConstants {
    // AprilTag layout
    public static final AprilTagFieldLayout aprilTagLayout;

    static {
      try {
        aprilTagLayout =
            new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath() + "/apriltags.json");
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }

    // Camera names, must match names configured on coprocessor
    public static final String leftCameraName = "leftCamera";
    public static final String rightCameraName = "rightCamera";
    public static final String turretCameraName = "turretCamera";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static final Transform3d robotToLeftCamera =
        new Transform3d(
            Units.inchesToMeters(9.839),
            Units.inchesToMeters(10.574),
            Units.inchesToMeters(5.952),
            new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(20)));
    public static final Transform3d robotToRightCamera =
        new Transform3d(
            Units.inchesToMeters(9.839),
            Units.inchesToMeters(-10.574),
            Units.inchesToMeters(5.952),
            new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-20)));

    public static final Transform3d turretToCamera =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(5.6875), 0.0, Units.inchesToMeters(.375)),
            new Rotation3d(0.0, Units.degreesToRadians(-30), 0.0));

    public static Transform3d getRobotToTurretCamera(Rotation2d turretYaw) {
      Pose3d turretCenter = Pose3d.kZero.transformBy(TurretConstants.ROBOT_TO_TURRET_TRANSFORM);
      Pose3d cameraPose =
          turretCenter
              .transformBy(turretToCamera)
              .rotateAround(
                  turretCenter.getTranslation(), new Rotation3d(0.0, 0.0, turretYaw.getRadians()));
      return new Transform3d(Pose3d.kZero, cameraPose);
    }

    public static final double maxAmbiguity = 0.1;
    public static final double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] cameraStdDevFactors =
        new double[] {
          1.0, // Left camera
          1.0, // Right camera
          1.0 // Turret camera
        };

    // Multipliers to apply for MegaTag 2 observations
    public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }
}
