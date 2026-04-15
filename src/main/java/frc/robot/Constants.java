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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.turret.TurretCalculator;

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
    /** Gear ratio between pivot motor output shaft and the mechanism. */
    public static final double PIVOT_GEAR_RATIO =
        (64.0 / 14) * (64.0 / 16) * (36.0 / 14); // 14:64 gear, 16:64 gear, 14:36 chain;

    /** Gear ratio between roller motor output shaft and the roller mechanism. */
    public static final double ROLLER_GEAR_RATIO = 1.0;

    public static final int PIVOT_FOLLOWER_CAN_ID = 41;
    public static final int PIVOT_CAN_ID = 49;
    public static final int ROLLER_CAN_ID = 48;
  }

  public static final class IndexerConstants {
    private IndexerConstants() {}

    public static final int INDEXER_ID = 42;
    public static final int CONVEYOR_CAN_ID = 50;
    public static final int LEFT_SIDE_ROLLER_CAD_ID = 45;
    public static final int RIGHT_SIDE_ROLLER_CAD_ID = 44;
  }

  public static class TurretConstants {
    public static final int TURN_ID = 43;
    public static final int HOOD_ID = 47;
    public static final int FLYWHEEL_ID = 46;

    public static final double TURN_TO_TURRET_RATIO =
        (50.0 / 16) * (90.0 / 10); // 16:50 gear, 10:90 gear
    public static final double HOOD_MOTOR_RATIO =
        (24.0 / 12) * (188.0 / 10); // 2:1 belt, 180:10 rack
    public static final double FLYWHEEL_MOTOR_RATIO = 0.5; // 1:2

    public static final Slot0Configs TURN_GAINS =
        new Slot0Configs().withKP(400).withKD(0.1).withKS(2);

    public static final Slot0Configs HOOD_GAINS =
        new Slot0Configs().withKP(256).withKD(5).withKS(0.28);

    public static final Slot0Configs FLYWHEEL_GAINS =
        new Slot0Configs().withKP(8).withKD(0.4).withKS(10).withKV(0.3);

    public static final CurrentLimitsConfigs TURN_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(30);

    public static final CurrentLimitsConfigs HOOD_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(30);

    public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS =
        new CurrentLimitsConfigs().withStatorCurrentLimit(80);

    public static final MotorOutputConfigs TURN_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final MotorOutputConfigs HOOD_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final MotorOutputConfigs FLYWHEEL_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    public static final FeedbackConfigs FLYWHEEL_FEEDBACK_CONFIGS =
        new FeedbackConfigs()
            .withVelocityFilterTimeConstant(Seconds.of(0.01))
            .withRotorToSensorRatio(FLYWHEEL_MOTOR_RATIO);

    public static final Current BANG_BANG_AMPS = Amps.of(100);

    public static final Distance DISTANCE_ABOVE_FUNNEL =
        Inches.of(20); // how high to clear the funnel
    public static final Distance APEX = Inches.of(130);
    public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
        new Transform3d(
            new Translation3d(Inches.zero(), Inches.of(-5.5), Inches.of(9.2)), Rotation3d.kZero);
    public static final Distance FLYWHEEL_RADIUS = Inches.of(2);
    public static final Distance SHOOT_RADIUS = Inches.of(1);
    public static final int LOOKAHEAD_ITERATIONS = 3;

    public static final Angle MIN_TURN_ANGLE = Degrees.of(-90);
    public static final Angle MAX_TURN_ANGLE = Degrees.of(90);
    public static final Angle TURNAROUND_ZONE = Degrees.of(30);

    public static final Angle MIN_HOOD_ANGLE = Degrees.of(21.154316);
    public static final Angle MAX_HOOD_ANGLE = Degrees.of(50);

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
    public static final Translation3d PASSING_SPOT_CENTER =
        new Translation3d(
            Inches.of(90), Constants.FieldConstants.FIELD_WIDTH.div(2), Inches.zero());
    public static final Translation3d PASSING_SPOT_RIGHT =
        new Translation3d(
            Inches.of(90),
            Constants.FieldConstants.FIELD_WIDTH.div(2).minus(Inches.of(85)),
            Inches.zero());

    public static final InterpolatingTreeMap<Double, TurretCalculator.ShotData> SHOT_MAP =
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), TurretCalculator.ShotData::interpolate);

    public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

    static { // TODO Tune these
      //      SHOT_MAP.put(5.34, new TurretCalculator.ShotData(RPM.of(2770), Degrees.of(27)));
      //      TOF_MAP.put(5.34, 1.30);
      //
      //      SHOT_MAP.put(4.90, new TurretCalculator.ShotData(RPM.of(2740), Degrees.of(26)));
      //      TOF_MAP.put(4.90, 1.42);
      //
      //      SHOT_MAP.put(4.44, new TurretCalculator.ShotData(RPM.of(2725), Degrees.of(25.5)));
      //      TOF_MAP.put(4.44, 1.34);
      //
      //      SHOT_MAP.put(4.05, new TurretCalculator.ShotData(RPM.of(2725), Degrees.of(25)));
      //      TOF_MAP.put(4.05, 1.36);
      //
      //      SHOT_MAP.put(3.74, new TurretCalculator.ShotData(RPM.of(2660), Degrees.of(24)));
      //      TOF_MAP.put(3.74, 1.21);
      //
      //      SHOT_MAP.put(3.42, new TurretCalculator.ShotData(RPM.of(2600), Degrees.of(23)));
      //      TOF_MAP.put(3.42, 1.40);
      //
      //      SHOT_MAP.put(3.06, new TurretCalculator.ShotData(RPM.of(2510), Degrees.of(22)));
      //      TOF_MAP.put(3.06, 1.38);
      //
      //      SHOT_MAP.put(2.73, new TurretCalculator.ShotData(RPM.of(2410), Degrees.of(20.5)));
      //      TOF_MAP.put(2.73, 1.34);
      //
      //      SHOT_MAP.put(2.45, new TurretCalculator.ShotData(RPM.of(2360), Degrees.of(19.5)));
      //      TOF_MAP.put(2.45, 1.28);
      //
      //      SHOT_MAP.put(2.14, new TurretCalculator.ShotData(RPM.of(2310), Degrees.of(18)));
      //      TOF_MAP.put(2.14, 1.31);
      //
      //      SHOT_MAP.put(1.86, new TurretCalculator.ShotData(RPM.of(2260), Degrees.of(17)));
      //      TOF_MAP.put(1.86, 1.24);
      //
      //      SHOT_MAP.put(1.55, new TurretCalculator.ShotData(RPM.of(2235), Degrees.of(15)));
      //      TOF_MAP.put(1.55, 1.23);
      SHOT_MAP.put(2.0, new TurretCalculator.ShotData(RPM.of(2750), Degrees.of(22)));
      TOF_MAP.put(2.0, 1.05);
      SHOT_MAP.put(3.2, new TurretCalculator.ShotData(RPM.of(3100), Degrees.of(22)));
      TOF_MAP.put(3.2, 1.16);
      SHOT_MAP.put(5.0, new TurretCalculator.ShotData(RPM.of(3550), Degrees.of(28)));
      TOF_MAP.put(5.0, 1.33);
    }

    public static final Time ACTIVE_PRESHOOT_TIME = Seconds.of(2);
  }

  public static final int OPERATOR_KEYBOARD_PORT = 2;

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
}
