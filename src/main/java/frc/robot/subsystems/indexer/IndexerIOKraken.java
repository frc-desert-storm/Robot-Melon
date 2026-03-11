package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  private final TalonFX leftRollerMotor;
  private final TalonFX rightRollerMotor;
  private final TalonFX indexerMotor;

  // ── Control request ───────────────────────────────────────────────────────
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  // ── Status signals ────────────────────────────────────────────────────────
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Current> followerCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private double setpointRPM = 0;

  private final VelocityVoltage velocityReq = new VelocityVoltage(0).withEnableFOC(true);

  /**
   * Constructs the TalonFX indexer IO.
   *
   * @param leftRollerMotorId CAN ID for the indexer leader motor
   * @param rightRollerMotorId CAN ID for the indexer follower motor (direction will be flipped)
   * @param canbus CANivore bus name, or empty string for RIO CAN
   */
  public IndexerIOKraken(int leftRollerMotorId, int rightRollerMotorId, int indexerId) {
    leftRollerMotor = new TalonFX(leftRollerMotorId);
    rightRollerMotor = new TalonFX(rightRollerMotorId);
    indexerMotor = new TalonFX(indexerId);

    // ── Leader configuration ──────────────────────────────────────────────
    var leftRollerCfg = new TalonFXConfiguration();
    leftRollerCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftRollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftRollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    leftRollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftRollerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    leftRollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    leftRollerCfg.Slot0.kP = 10.0;
    leftRollerCfg.Slot0.kI = 0.2;

    leftRollerMotor.getConfigurator().apply(leftRollerCfg);
    // ── Follower configuration ────────────────────────────────────────────
    var rightRollerCfg = new TalonFXConfiguration();
    rightRollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightRollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    rightRollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightRollerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    rightRollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rightRollerCfg.Slot0.kP = 10.0;
    rightRollerCfg.Slot0.kI = 0.2;

    rightRollerMotor.getConfigurator().apply(rightRollerCfg);

    // ── Follower2 configuration ────────────────────────────────────────────
    var indexerCfg = new TalonFXConfiguration();
    indexerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    indexerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    indexerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    indexerCfg.Slot0.kP = 8.0;

    indexerMotor.getConfigurator().apply(rightRollerCfg);

    // Follower with direction flipped (oppose leader direction = true)
    rightRollerMotor.setControl(new Follower(leftRollerMotorId, MotorAlignmentValue.Opposed));

    // ── Status signal registration ────────────────────────────────────────
    leaderVelocity = leftRollerMotor.getVelocity();
    leaderAppliedVolts = leftRollerMotor.getMotorVoltage();
    leaderCurrent = leftRollerMotor.getSupplyCurrent();
    leaderTemp = leftRollerMotor.getDeviceTemp();
    followerCurrent = rightRollerMotor.getSupplyCurrent();
    followerTemp = rightRollerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        leaderTemp,
        followerCurrent,
        followerTemp);

    leftRollerMotor.optimizeBusUtilization();
    rightRollerMotor.optimizeBusUtilization();
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

    inputs.setpointRPM = setpointRPM;
  }

  @Override
  public void setSideRollerVoltage(double volts) {
    leftRollerMotor.setControl(voltageRequest.withOutput(volts));
    // Follower automatically opposes the leader's direction via hardware Follower control
  }

  @Override
  public void setIndexerVoltage(double volts) {
    indexerMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setSideRollersVelocityRPM(double rpm) {
    setpointRPM = rpm;
    leftRollerMotor.setControl(velocityReq.withVelocity(rpm / 60.0));
  }

  @Override
  public void setIndexerRPM(double rpm) {
    setpointRPM = rpm;
    indexerMotor.setControl(velocityReq.withVelocity(rpm / 60.0));
  }
}
