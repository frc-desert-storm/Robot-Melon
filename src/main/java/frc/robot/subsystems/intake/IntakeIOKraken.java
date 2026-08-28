package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;

public class IntakeIOKraken implements IntakeIO {

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
    extensionCfg.MotorOutput = EXTENSION_OUTPUT_CONFIGS;
    extensionCfg.Feedback.SensorToMechanismRatio = EXTENSION_GEAR_RATIO;
    extensionCfg.Slot0 = EXTENSION_GAINS;
    extensionCfg.MotionMagic = EXTENSION_MOTION_MAGIC;
    extensionCfg.CurrentLimits = EXTENSION_CURRENT_LIMITS;
    PhoenixUtil.tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionCfg, 0.25));

    var extensionLeftCfg = new TalonFXConfiguration();
    extensionLeftCfg.MotorOutput = EXTENSION_LEFT_OUTPUT_CONFIGS;
    extensionLeftCfg.Feedback.SensorToMechanismRatio = EXTENSION_GEAR_RATIO;
    extensionLeftCfg.Slot0 = EXTENSION_LEFT_GAINS;
    extensionLeftCfg.MotionMagic = EXTENSION_LEFT_MOTION_MAGIC;
    extensionLeftCfg.CurrentLimits = EXTENSION_LEFT_CURRENT_LIMITS;
    PhoenixUtil.tryUntilOk(
        5, () -> extensionLeftMotor.getConfigurator().apply(extensionLeftCfg, 0.25));

    // ── Extension Roller configuration ──────────────────────────────────────────────

    var rollerCfg = new TalonFXConfiguration();
    rollerCfg.MotorOutput = ROLLER_OUTPUT_CONFIGS;
    rollerCfg.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;
    rollerCfg.CurrentLimits = ROLLER_CURRENT_LIMITS;
    rollerCfg.Slot0 = ROLLER_GAINS;

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
  public void setExtensionVoltage(Voltage voltage) {
    extensionMotor.setVoltage(voltage.in(Volts));
    extensionLeftMotor.setVoltage(voltage.in(Volts));
  }

  @Override
  public void zeroExtensionDistance() {
    extensionLeftMotor.setPosition(DistanceToAngle(INTAKING_POSE));
    extensionMotor.setPosition(DistanceToAngle(INTAKING_POSE));
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
