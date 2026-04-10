package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;

/**
 * Real-robot implementation of {@link IndexerIO} using two Kraken X60 (TalonFX) motors.
 *
 * <p>The follower runs with its direction <em>inverted</em> relative to the leader so that both
 * motors physically push fuel in the same linear direction when mounted on opposite sides of the
 * feed path.
 */
public class IndexerIOKraken implements IndexerIO {

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final TalonFX indexerMotor;
  private final TalonFX conveyorMotor;

  // ── Status signals ────────────────────────────────────────────────────────

  private final StatusSignal<AngularVelocity> indexerRollerVelocity;
  private final StatusSignal<Voltage> indexerRollerAppliedVolts;
  private final StatusSignal<Current> indexerRollerCurrent;

  private final StatusSignal<AngularVelocity> conveyorRollerVelocity;
  private final StatusSignal<Voltage> conveyorRollerAppliedVolts;
  private final StatusSignal<Current> conveyorRollerCurrent;

  private final VelocityVoltage velocityReq = new VelocityVoltage(0).withEnableFOC(true);

  private final NeutralOut neutralOut = new NeutralOut();

  public IndexerIOKraken() {
    indexerMotor = new TalonFX(INDEXER_ID, TunerConstants.kCANBus);
    conveyorMotor = new TalonFX(CONVEYOR_CAN_ID, TunerConstants.kCANBus);

    var indexerCfg = new TalonFXConfiguration();
    indexerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    indexerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerCfg.CurrentLimits.StatorCurrentLimit = 40.0;
    indexerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    indexerCfg.Slot0.kP = 8.0;

    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerCfg, 0.25));

    var conveyorRollerCfg = new TalonFXConfiguration();
    conveyorRollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    conveyorRollerCfg.CurrentLimits.SupplyCurrentLimit = 35.0;
    conveyorRollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    conveyorRollerCfg.CurrentLimits.StatorCurrentLimit = 50.0;
    conveyorRollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    conveyorRollerCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    conveyorRollerCfg.Slot0.kP = 10.0;

    PhoenixUtil.tryUntilOk(5, () -> conveyorMotor.getConfigurator().apply(conveyorRollerCfg, 0.25));

    indexerRollerVelocity = indexerMotor.getVelocity();
    indexerRollerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerRollerCurrent = indexerMotor.getSupplyCurrent();

    conveyorRollerVelocity = conveyorMotor.getVelocity();
    conveyorRollerAppliedVolts = conveyorMotor.getMotorVoltage();
    conveyorRollerCurrent = conveyorMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerRollerVelocity,
        indexerRollerAppliedVolts,
        indexerRollerCurrent,
        conveyorRollerVelocity,
        conveyorRollerAppliedVolts,
        conveyorRollerCurrent);

    indexerMotor.optimizeBusUtilization();
    conveyorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerRollerConnected =
        BaseStatusSignal.isAllGood(
            indexerRollerVelocity, indexerRollerAppliedVolts, indexerRollerCurrent);
    inputs.indexerRollerSpeed = indexerRollerVelocity.getValue();
    inputs.indexerRollerAppliedVolts = indexerRollerAppliedVolts.getValue();
    inputs.indexerRollerCurrent = indexerRollerCurrent.getValue();

    inputs.conveyorRollerConnected =
        BaseStatusSignal.isAllGood(
            conveyorRollerVelocity, conveyorRollerAppliedVolts, conveyorRollerCurrent);
    inputs.conveyorRollerSpeed = conveyorRollerVelocity.getValue();
    inputs.conveyorRollerAppliedVolts = conveyorRollerAppliedVolts.getValue();
    inputs.conveyorRollerCurrent = conveyorRollerCurrent.getValue();
  }

  @Override
  public void setIndexerSpeed(AngularVelocity speed) {
    indexerMotor.setControl(velocityReq.withVelocity(speed));
  }

  @Override
  public void setConveyorSpeed(AngularVelocity speed) {
    conveyorMotor.setControl(velocityReq.withVelocity(speed));
  }

  @Override
  public void stopIndexer() {
    indexerMotor.setControl(neutralOut);
  }

  @Override
  public void stopConveyor() {
    conveyorMotor.setControl(neutralOut);
  }
}
