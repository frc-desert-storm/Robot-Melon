package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;

/**
 * Real-robot implementation of {@link IntakeIO} using three Kraken X60 (TalonFX) motors.
 *
 * <ul>
 *   <li>Motor 1 (extensionMotor) – Extension arm position control
 *   <li>Motor 2 (rollerMotor) – Roller leader, voltage control
 * </ul>
 */
public class IntakeIOKraken implements IntakeIO {

  // ── Configurable constants ───────────────────────────────────────────────

  // MotionMagic gains for extension (tune to robot)
  private static final double EXTENSION_kP = 24.0;
  private static final double EXTENSION_kI = 0.0;
  private static final double EXTENSION_kD = 1.5;
  private static final double EXTENSION_kS = 0.0;
  private static final double EXTENSION_kV = 0.4;
  private static final double EXTENSION_kA = 0.0;
  private static final double EXTENSION_kG = 0.0;
  private static final double EXTENSION_CRUISE_RPS = 160.0; // motor rotations per second
  private static final double EXTENSION_ACCEL_RPS2 = 360.0; // motor rotations per second²
  private static final double EXTENSION_JERK_RPS3 = 1600.0;

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final TalonFX extensionMotor;
  private final TalonFX extensionLeftMotor;
  private final TalonFX rollerMotor;

  // ── Control requests ─────────────────────────────────────────────────────
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(true);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

  private final NeutralOut neutralOut = new NeutralOut();

  // ── Status signals ────────────────────────────────────────────────────────
  private final StatusSignal<Angle> extensionPosition;
  private final StatusSignal<AngularVelocity> extensionVelocity;
  private final StatusSignal<Voltage> extensionAppliedVolts;
  private final StatusSignal<Current> extensionCurrent;
  private final StatusSignal<Boolean> extensionAtGoal;

  private final StatusSignal<Angle> extensionLeftPosition;
  private final StatusSignal<AngularVelocity> extensionLeftVelocity;
  private final StatusSignal<Voltage> extensionLeftAppliedVolts;
  private final StatusSignal<Current> extensionLeftCurrent;
  private final StatusSignal<Boolean> extensionLeftAtGoal;

  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;

  public IntakeIOKraken() {
    extensionMotor = new TalonFX(EXTENSION_CAN_ID, TunerConstants.kCANBus);
    extensionLeftMotor = new TalonFX(EXTENSION_FOLLOWER_CAN_ID, TunerConstants.kCANBus);
    rollerMotor = new TalonFX(ROLLER_CAN_ID, TunerConstants.kCANBus);

    // ── Extension configuration ────────────────────────────────────────────────
    var extensionCfg = new TalonFXConfiguration();
    extensionCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    extensionCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    extensionCfg.Feedback.SensorToMechanismRatio = EXTENSION_GEAR_RATIO;
    extensionCfg.Slot0.kP = EXTENSION_kP;
    extensionCfg.Slot0.kI = EXTENSION_kI;
    extensionCfg.Slot0.kD = EXTENSION_kD;
    extensionCfg.Slot0.kS = EXTENSION_kS;
    extensionCfg.Slot0.kV = EXTENSION_kV;
    extensionCfg.Slot0.kA = EXTENSION_kA;
    extensionCfg.Slot0.kG = EXTENSION_kG;
    extensionCfg.MotionMagic.MotionMagicCruiseVelocity = EXTENSION_CRUISE_RPS;
    extensionCfg.MotionMagic.MotionMagicAcceleration = EXTENSION_ACCEL_RPS2;
    extensionCfg.MotionMagic.MotionMagicJerk = EXTENSION_JERK_RPS3;
    extensionCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    extensionCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionCfg.CurrentLimits.StatorCurrentLimit = 20.0;
    extensionCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    PhoenixUtil.tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionCfg, 0.25));

    var extensionLeftCfg = new TalonFXConfiguration();
    extensionLeftCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    extensionLeftCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    extensionLeftCfg.Feedback.SensorToMechanismRatio = EXTENSION_GEAR_RATIO;
    extensionLeftCfg.Slot0.kP = EXTENSION_kP;
    extensionLeftCfg.Slot0.kI = EXTENSION_kI;
    extensionLeftCfg.Slot0.kD = EXTENSION_kD;
    extensionLeftCfg.Slot0.kS = EXTENSION_kS;
    extensionLeftCfg.Slot0.kV = EXTENSION_kV;
    extensionLeftCfg.Slot0.kA = EXTENSION_kA;
    extensionLeftCfg.Slot0.kG = EXTENSION_kG;
    extensionLeftCfg.MotionMagic.MotionMagicCruiseVelocity = EXTENSION_CRUISE_RPS;
    extensionLeftCfg.MotionMagic.MotionMagicAcceleration = EXTENSION_ACCEL_RPS2;
    extensionLeftCfg.MotionMagic.MotionMagicJerk = EXTENSION_JERK_RPS3;
    extensionLeftCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    extensionLeftCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionLeftCfg.CurrentLimits.StatorCurrentLimit = 20.0;
    extensionLeftCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionLeftCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> extensionLeftMotor.getConfigurator().apply(extensionLeftCfg, 0.25));

    // ── Extension Roller configuration ──────────────────────────────────────────────

    var rollerCfg = new TalonFXConfiguration();
    rollerCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerCfg.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;
    rollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerCfg.Slot0.kP = 0.2;
    rollerCfg.Slot0.kV = 1.0;

    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerCfg, 0.25));

    // ── Status signal registration ────────────────────────────────────────
    extensionPosition = extensionMotor.getPosition();
    extensionVelocity = extensionMotor.getVelocity();
    extensionAppliedVolts = extensionMotor.getMotorVoltage();
    extensionCurrent = extensionMotor.getSupplyCurrent();
    extensionAtGoal = extensionMotor.getMotionMagicAtTarget();

    extensionLeftPosition = extensionLeftMotor.getPosition();
    extensionLeftVelocity = extensionLeftMotor.getVelocity();
    extensionLeftAppliedVolts = extensionLeftMotor.getMotorVoltage();
    extensionLeftCurrent = extensionLeftMotor.getSupplyCurrent();
    extensionLeftAtGoal = extensionLeftMotor.getMotionMagicAtTarget();

    rollerVelocity = rollerMotor.getVelocity();
    rollerAppliedVolts = rollerMotor.getMotorVoltage();
    rollerCurrent = rollerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        extensionPosition,
        extensionVelocity,
        extensionAppliedVolts,
        extensionCurrent,
        extensionLeftPosition,
        extensionLeftVelocity,
        extensionLeftAppliedVolts,
        extensionLeftCurrent,
        extensionAtGoal,
        extensionLeftAtGoal,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent);

    extensionMotor.optimizeBusUtilization();
    extensionLeftMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();

    extensionMotor.setPosition(0);
    extensionLeftMotor.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extensionLeftMotorConnected =
        BaseStatusSignal.refreshAll(
                extensionLeftPosition,
                extensionLeftVelocity,
                extensionLeftAppliedVolts,
                extensionLeftCurrent)
            .isOK();
    inputs.extensionMotorConnected =
        BaseStatusSignal.refreshAll(
                extensionPosition, extensionVelocity, extensionAppliedVolts, extensionCurrent)
            .isOK();
    inputs.rollerMotorConnected =
        BaseStatusSignal.refreshAll(rollerAppliedVolts, rollerVelocity, rollerCurrent).isOK();

    inputs.extensionPosition = AngleToDistance(extensionPosition.getValue());
    inputs.extensionVelocity = AngularVelocityToLinearVelocity(extensionVelocity.getValue());
    inputs.extensionAppliedVolts = extensionAppliedVolts.getValue();
    inputs.extensionCurrentAmps = extensionCurrent.getValue();
    inputs.extensionAtGoal = extensionAtGoal.getValue();

    inputs.extensionLeftPosition = AngleToDistance(extensionLeftPosition.getValue());
    inputs.extensionLeftVelocity =
        AngularVelocityToLinearVelocity(extensionLeftVelocity.getValue());
    inputs.extensionLeftAppliedVolts = extensionLeftAppliedVolts.getValue();
    inputs.extensionLeftCurrentAmps = extensionLeftCurrent.getValue();
    inputs.extensionLeftAtGoal = extensionLeftAtGoal.getValue();

    inputs.rollerVelocity = rollerVelocity.getValue();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValue();
    inputs.rollerCurrentAmps = rollerCurrent.getValue();
  }

  @Override
  public void setExtensionDistance(Distance distance) {
    var request = mmRequest.withPosition(DistanceToAngle(distance));
    extensionMotor.setControl(request);
    extensionLeftMotor.setControl(request);
  }

  @Override
  public void setRollerSpeed(AngularVelocity speed) {
    rollerMotor.setControl(rollerVelocityRequest.withVelocity((speed)));
  }

  @Override
  public void stopExtension() {
    extensionMotor.setControl(neutralOut);
    extensionLeftMotor.setControl(neutralOut);
  }

  @Override
  public void stopRoller() {
    rollerMotor.setControl(neutralOut);
  }
}
