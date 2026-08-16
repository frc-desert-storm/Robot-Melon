package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;

public class IndexerIOKraken implements IndexerIO {

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final TalonFX indexerMotor;

  // ── Status signals ────────────────────────────────────────────────────────
  private final StatusSignal<AngularVelocity> indexerRollerVelocity;
  private final StatusSignal<Voltage> indexerRollerAppliedVolts;
  private final StatusSignal<Current> indexerRollerCurrent;

  private final VelocityVoltage velocityReq = new VelocityVoltage(0).withEnableFOC(true);

  private final NeutralOut neutralOut = new NeutralOut();

  public IndexerIOKraken() {
    indexerMotor = new TalonFX(INDEXER_ID, TunerConstants.kCANBus);

    var indexerCfg = new TalonFXConfiguration();
    indexerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerCfg.CurrentLimits = INDEXER_CURRENT_LIMITS;

    indexerCfg.Slot0 = INDEXER_GAINS;

    indexerCfg.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(INDEXER_GEAR_RATIO);

    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerCfg, 0.25));

    // ── Status signal registration ────────────────────────────────────────
    indexerRollerVelocity = indexerMotor.getVelocity();
    indexerRollerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerRollerCurrent = indexerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, indexerRollerVelocity, indexerRollerAppliedVolts, indexerRollerCurrent);

    indexerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerRollerConnected =
        BaseStatusSignal.refreshAll(
                indexerRollerVelocity, indexerRollerAppliedVolts, indexerRollerCurrent)
            .isOK();
    inputs.indexerRollerSpeed = indexerRollerVelocity.getValue();
    inputs.indexerRollerAppliedVolts = indexerRollerAppliedVolts.getValue();
    inputs.indexerRollerCurrent = indexerRollerCurrent.getValue();
  }

  @Override
  public void setIndexerSpeed(AngularVelocity speed) {
    indexerMotor.setControl(velocityReq.withVelocity(speed));
  }

  @Override
  public void stopIndexer() {
    indexerMotor.setControl(neutralOut);
  }
}
