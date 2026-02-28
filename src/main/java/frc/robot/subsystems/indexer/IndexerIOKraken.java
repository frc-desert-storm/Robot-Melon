package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Real-robot implementation of {@link IndexerIO} using two Kraken X60 (TalonFX) motors.
 *
 * <p>The follower runs with its direction <em>inverted</em> relative to the leader so that both
 * motors physically push fuel in the same linear direction when mounted on opposite sides of the
 * feed path.
 */
public class IndexerIOKraken implements IndexerIO {

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  // ── Control request ───────────────────────────────────────────────────────
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  // ── Status signals ────────────────────────────────────────────────────────
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Current> followerCurrent;
  private final StatusSignal<Temperature> followerTemp;

  /**
   * Constructs the TalonFX indexer IO.
   *
   * @param leaderCanId CAN ID for the indexer leader motor
   * @param followerCanId CAN ID for the indexer follower motor (direction will be flipped)
   * @param canbus CANivore bus name, or empty string for RIO CAN
   */
  public IndexerIOKraken(int leaderCanId, int followerCanId, String canbus) {
    leaderMotor = new TalonFX(leaderCanId, canbus);
    followerMotor = new TalonFX(followerCanId, canbus);

    // ── Leader configuration ──────────────────────────────────────────────
    var leaderCfg = new TalonFXConfiguration();
    leaderCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    leaderCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderMotor.getConfigurator().apply(leaderCfg);

    // ── Follower configuration ────────────────────────────────────────────
    var followerCfg = new TalonFXConfiguration();
    followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    followerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    followerCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    followerMotor.getConfigurator().apply(followerCfg);

    // Follower with direction flipped (oppose leader direction = true)
    followerMotor.setControl(new Follower(leaderCanId, MotorAlignmentValue.Opposed));

    // ── Status signal registration ────────────────────────────────────────
    leaderVelocity = leaderMotor.getVelocity();
    leaderAppliedVolts = leaderMotor.getMotorVoltage();
    leaderCurrent = leaderMotor.getSupplyCurrent();
    leaderTemp = leaderMotor.getDeviceTemp();
    followerCurrent = followerMotor.getSupplyCurrent();
    followerTemp = followerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        leaderTemp,
        followerCurrent,
        followerTemp);

    leaderMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(leaderVelocity, leaderAppliedVolts, leaderCurrent, leaderTemp)
            .isOK();
    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(followerCurrent, followerTemp).isOK();

    inputs.leaderVelocityRpm = leaderVelocity.getValueAsDouble() * 60.0;
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();
    inputs.leaderTempCelsius = leaderTemp.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
    inputs.followerTempCelsius = followerTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(voltageRequest.withOutput(volts));
    // Follower automatically opposes the leader's direction via hardware Follower control
  }
}
