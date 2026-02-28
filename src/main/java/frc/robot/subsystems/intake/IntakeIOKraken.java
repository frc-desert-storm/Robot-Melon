package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Real-robot implementation of {@link IntakeIO} using three Kraken X60 (TalonFX) motors.
 *
 * <ul>
 *   <li>Motor 1 (liftMotor) – Lift arm position control
 *   <li>Motor 2 (rollerMotor) – Roller leader, voltage control
 *   <li>Motor 3 (conveyorMotor)– Conveyor follower, mirrors roller leader
 * </ul>
 */
public class IntakeIOKraken implements IntakeIO {

  // ── Configurable constants ────────────────────────────────────────────────
  /** Gear ratio between lift motor output shaft and the mechanism. */
  public static final double LIFT_GEAR_RATIO = 1.0;

  /** Gear ratio between roller motor output shaft and the roller mechanism. */
  public static final double ROLLER_GEAR_RATIO = 1.0;

  // MotionMagic gains for lift (tune to robot)
  private static final double LIFT_kP = 24.0;
  private static final double LIFT_kI = 0.0;
  private static final double LIFT_kD = 0.4;
  private static final double LIFT_kS = 0.25;
  private static final double LIFT_kV = 0.12;
  private static final double LIFT_kA = 0.01;
  private static final double LIFT_CRUISE_RPS = 80.0; // motor rotations per second
  private static final double LIFT_ACCEL_RPS2 = 160.0; // motor rotations per second²
  private static final double LIFT_JERK_RPS3 = 1600.0;

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final TalonFX liftMotor;
  private final TalonFX rollerMotor;
  private final TalonFX conveyorMotor;

  // ── Control requests ─────────────────────────────────────────────────────
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut liftVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

  // ── Status signals ────────────────────────────────────────────────────────
  private final StatusSignal<Angle> liftPosition;
  private final StatusSignal<AngularVelocity> liftVelocity;
  private final StatusSignal<Voltage> liftAppliedVolts;
  private final StatusSignal<Current> liftCurrent;
  private final StatusSignal<Temperature> liftTemp;

  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<Temperature> rollerTemp;
  private final StatusSignal<Current> conveyorCurrent;
  private final StatusSignal<Temperature> conveyorTemp;

  /**
   * Constructs the TalonFX intake IO.
   *
   * @param liftCanId CAN ID for the lift motor
   * @param rollerCanId CAN ID for the roller leader motor
   * @param conveyorCanId CAN ID for the conveyor follower motor
   * @param canbus CANivore bus name, or empty string for RIO CAN
   */
  public IntakeIOKraken(int liftCanId, int rollerCanId, int conveyorCanId, String canbus) {
    liftMotor = new TalonFX(liftCanId, canbus);
    rollerMotor = new TalonFX(rollerCanId, canbus);
    conveyorMotor = new TalonFX(conveyorCanId, canbus);

    // ── Lift configuration ────────────────────────────────────────────────
    var liftCfg = new TalonFXConfiguration();
    liftCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    liftCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    liftCfg.Feedback.SensorToMechanismRatio = LIFT_GEAR_RATIO;
    liftCfg.Slot0.kP = LIFT_kP;
    liftCfg.Slot0.kI = LIFT_kI;
    liftCfg.Slot0.kD = LIFT_kD;
    liftCfg.Slot0.kS = LIFT_kS;
    liftCfg.Slot0.kV = LIFT_kV;
    liftCfg.Slot0.kA = LIFT_kA;
    liftCfg.MotionMagic.MotionMagicCruiseVelocity = LIFT_CRUISE_RPS;
    liftCfg.MotionMagic.MotionMagicAcceleration = LIFT_ACCEL_RPS2;
    liftCfg.MotionMagic.MotionMagicJerk = LIFT_JERK_RPS3;
    liftCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    liftCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    liftCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    liftCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    liftMotor.getConfigurator().apply(liftCfg);

    // ── Roller configuration ──────────────────────────────────────────────
    var rollerCfg = new TalonFXConfiguration();
    rollerCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerCfg.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;
    rollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    rollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotor.getConfigurator().apply(rollerCfg);

    // ── Conveyor (follower) configuration ────────────────────────────────
    var conveyorCfg = new TalonFXConfiguration();
    conveyorCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    conveyorCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    conveyorCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    conveyorCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    conveyorCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    conveyorMotor.getConfigurator().apply(conveyorCfg);

    // Conveyor follows roller in the same direction
    conveyorMotor.setControl(new Follower(rollerCanId, MotorAlignmentValue.Aligned));

    // ── Status signal registration ────────────────────────────────────────
    liftPosition = liftMotor.getPosition();
    liftVelocity = liftMotor.getVelocity();
    liftAppliedVolts = liftMotor.getMotorVoltage();
    liftCurrent = liftMotor.getSupplyCurrent();
    liftTemp = liftMotor.getDeviceTemp();

    rollerVelocity = rollerMotor.getVelocity();
    rollerAppliedVolts = rollerMotor.getMotorVoltage();
    rollerCurrent = rollerMotor.getSupplyCurrent();
    rollerTemp = rollerMotor.getDeviceTemp();
    conveyorCurrent = conveyorMotor.getSupplyCurrent();
    conveyorTemp = conveyorMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        liftPosition,
        liftVelocity,
        liftAppliedVolts,
        liftCurrent,
        liftTemp,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent,
        rollerTemp,
        conveyorCurrent,
        conveyorTemp);

    liftMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
    conveyorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.liftMotorConnected =
        BaseStatusSignal.refreshAll(
                liftPosition, liftVelocity, liftAppliedVolts, liftCurrent, liftTemp)
            .isOK();
    inputs.rollerMotorConnected =
        BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts, rollerCurrent, rollerTemp)
            .isOK();
    inputs.conveyorMotorConnected =
        BaseStatusSignal.refreshAll(conveyorCurrent, conveyorTemp).isOK();

    inputs.liftPositionRot = liftPosition.getValueAsDouble();
    inputs.liftVelocityRpm = liftVelocity.getValueAsDouble() * 60.0;
    inputs.liftAppliedVolts = liftAppliedVolts.getValueAsDouble();
    inputs.liftCurrentAmps = liftCurrent.getValueAsDouble();
    inputs.liftTempCelsius = liftTemp.getValueAsDouble();

    inputs.rollerVelocityRpm = rollerVelocity.getValueAsDouble() * 60.0;
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
    inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();
    inputs.conveyorCurrentAmps = conveyorCurrent.getValueAsDouble();
    inputs.conveyorTempCelsius = conveyorTemp.getValueAsDouble();
  }

  @Override
  public void setLiftPosition(double positionRot) {
    liftMotor.setControl(mmRequest.withPosition(positionRot));
  }

  @Override
  public void setLiftVoltage(double volts) {
    liftMotor.setControl(liftVoltageRequest.withOutput(volts));
  }

  @Override
  public void setRollerVoltage(double volts) {
    // Conveyor follower automatically mirrors this command
    rollerMotor.setControl(rollerVoltageRequest.withOutput(volts));
  }

  @Override
  public void resetLiftEncoder() {
    liftMotor.setPosition(0.0);
  }

  @Override
  public void setLiftBrakeMode(boolean brake) {
    liftMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new com.ctre.phoenix6.configs.MotorOutputConfigs()
                        .withNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast)));
  }
}
