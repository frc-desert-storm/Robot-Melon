package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/**
 * Real-robot implementation of {@link IntakeIO} using three Kraken X60 (TalonFX) motors.
 *
 * <ul>
 *   <li>Motor 1 (pivotMotor) – Pivot arm position control
 *   <li>Motor 2 (rollerMotor) – Roller leader, voltage control
 * </ul>
 */
public class IntakeIOKraken implements IntakeIO {

  // ── Configurable constants ────────────────────────────────────────────────
  /** Gear ratio between pivot motor output shaft and the mechanism. */
  public static final double PIVOT_GEAR_RATIO = 9.0;

  /** Gear ratio between roller motor output shaft and the roller mechanism. */
  public static final double ROLLER_GEAR_RATIO = 1.0;

  // MotionMagic gains for pivot (tune to robot)
  private static final double PIVOT_kP = 40.0;
  private static final double PIVOT_kI = 0.0;
  private static final double PIVOT_kD = 0.0;
  private static final double PIVOT_kS = 0.5;
  private static final double PIVOT_kV = 0.8;
  private static final double PIVOT_kA = 0.0;
  private static final double PIVOT_kG = 0.45;
  private static final double PIVOT_CRUISE_RPS = 80.0; // motor rotations per second
  private static final double PIVOT_ACCEL_RPS2 = 160.0; // motor rotations per second²
  private static final double PIVOT_JERK_RPS3 = 1600.0;

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final TalonFX pivotMotor;
  private final TalonFX rollerMotor;

  // ── Control requests ─────────────────────────────────────────────────────
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(true);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

  private final NeutralOut neutralOut = new NeutralOut();

  // ── Status signals ────────────────────────────────────────────────────────
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;

  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;

  public IntakeIOKraken() {
    pivotMotor = new TalonFX(PIVOT_CAN_ID);
    rollerMotor = new TalonFX(ROLLER_CAN_ID);

    // ── Pivot configuration ────────────────────────────────────────────────
    var pivotCfg = new TalonFXConfiguration();
    pivotCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotCfg.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
    pivotCfg.Slot0.kP = PIVOT_kP;
    pivotCfg.Slot0.kI = PIVOT_kI;
    pivotCfg.Slot0.kD = PIVOT_kD;
    pivotCfg.Slot0.kS = PIVOT_kS;
    pivotCfg.Slot0.kV = PIVOT_kV;
    pivotCfg.Slot0.kA = PIVOT_kA;
    pivotCfg.Slot0.kG = PIVOT_kG;
    pivotCfg.MotionMagic.MotionMagicCruiseVelocity = PIVOT_CRUISE_RPS;
    pivotCfg.MotionMagic.MotionMagicAcceleration = PIVOT_ACCEL_RPS2;
    pivotCfg.MotionMagic.MotionMagicJerk = PIVOT_JERK_RPS3;
    pivotCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    pivotCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    pivotCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotCfg, 0.25));

    // ── Pivot Roller configuration ──────────────────────────────────────────────

    var rollerCfg = new TalonFXConfiguration();
    rollerCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerCfg.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;
    rollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerCfg.Slot0.kP = 8.0;
    rollerCfg.Slot0.kI = 0.2;

    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerCfg, 0.25));

    // ── Status signal registration ────────────────────────────────────────
    pivotPosition = pivotMotor.getPosition();
    pivotVelocity = pivotMotor.getVelocity();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getSupplyCurrent();

    rollerVelocity = rollerMotor.getVelocity();
    rollerAppliedVolts = rollerMotor.getMotorVoltage();
    rollerCurrent = rollerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent);

    pivotMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotMotorConnected =
        BaseStatusSignal.isAllGood(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    inputs.rollerMotorConnected =
        BaseStatusSignal.isAllGood(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);

    inputs.pivotPositionRot = pivotPosition.getValueAsDouble();
    inputs.pivotVelocityRpm = pivotVelocity.getValueAsDouble() * 60.0;
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();

    inputs.rollerVelocityRpm = rollerVelocity.getValueAsDouble() * 60.0;
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
    inputs.rollerRPM = ((rollerMotor.getVelocity().getValueAsDouble()) / (2 * Math.PI)) * 60;
  }

  @Override
  public void setPivotAngle(Angle angle) {
    pivotMotor.setControl(mmRequest.withPosition(angle));
  }

  @Override
  public void setRollerSpeed(AngularVelocity speed) {
    rollerMotor.setControl(rollerVelocityRequest.withVelocity((speed)));
  }

  @Override
  public void stopPivot() {
    pivotMotor.setControl(neutralOut);
  }

  @Override
  public void stopRoller() {
    rollerMotor.setControl(neutralOut);
  }
}
