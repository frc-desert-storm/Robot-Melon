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
  private final TalonFX leftRollerMotor;
  private final TalonFX rightRollerMotor;
  private final TalonFX indexerMotor;
  private final TalonFX conveyorMotor;

  // ── Status signals ────────────────────────────────────────────────────────
  private final StatusSignal<AngularVelocity> leftRollerVelocity;
  private final StatusSignal<Voltage> leftRollerAppliedVolts;
  private final StatusSignal<Current> leftRollerCurrent;

  private final StatusSignal<AngularVelocity> rightRollerVelocity;
  private final StatusSignal<Voltage> rightRollerAppliedVolts;
  private final StatusSignal<Current> rightRollerCurrent;

  private final StatusSignal<AngularVelocity> indexerRollerVelocity;
  private final StatusSignal<Voltage> indexerRollerAppliedVolts;
  private final StatusSignal<Current> indexerRollerCurrent;

  private final StatusSignal<AngularVelocity> conveyorRollerVelocity;
  private final StatusSignal<Voltage> conveyorRollerAppliedVolts;
  private final StatusSignal<Current> conveyorRollerCurrent;

  private final VelocityVoltage velocityReq = new VelocityVoltage(0).withEnableFOC(true);

  private final NeutralOut neutralOut = new NeutralOut();

  public IndexerIOKraken() {
    leftRollerMotor = new TalonFX(LEFT_ROLLER_ID);
    rightRollerMotor = new TalonFX(RIGHT_ROLLER_ID);
    indexerMotor = new TalonFX(INDEXER_ID);
    conveyorMotor = new TalonFX(CONVEYOR_CAN_ID);

    var leftRollerCfg = new TalonFXConfiguration();
    leftRollerCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftRollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftRollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    leftRollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftRollerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    leftRollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    leftRollerCfg.Slot0.kP = 10.0;

    PhoenixUtil.tryUntilOk(5, () -> leftRollerMotor.getConfigurator().apply(leftRollerCfg, 0.25));

    var rightRollerCfg = new TalonFXConfiguration();
    rightRollerCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightRollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightRollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    rightRollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightRollerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    rightRollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    rightRollerCfg.Slot0.kP = 10.0;

    PhoenixUtil.tryUntilOk(5, () -> rightRollerMotor.getConfigurator().apply(rightRollerCfg, 0.25));

    var indexerCfg = new TalonFXConfiguration();
    indexerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    indexerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    indexerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    indexerCfg.Slot0.kP = 8.0;

    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerCfg, 0.25));

    var conveyorRollerCfg = new TalonFXConfiguration();
    conveyorRollerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conveyorRollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    conveyorRollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    conveyorRollerCfg.CurrentLimits.StatorCurrentLimit = 60.0;
    conveyorRollerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    conveyorRollerCfg.Slot0.kP = 10.0;

    PhoenixUtil.tryUntilOk(5, () -> conveyorMotor.getConfigurator().apply(conveyorRollerCfg, 0.25));

    // ── Status signal registration ────────────────────────────────────────
    leftRollerVelocity = leftRollerMotor.getVelocity();
    leftRollerAppliedVolts = leftRollerMotor.getMotorVoltage();
    leftRollerCurrent = leftRollerMotor.getSupplyCurrent();

    rightRollerVelocity = rightRollerMotor.getVelocity();
    rightRollerAppliedVolts = rightRollerMotor.getMotorVoltage();
    rightRollerCurrent = rightRollerMotor.getSupplyCurrent();

    indexerRollerVelocity = indexerMotor.getVelocity();
    indexerRollerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerRollerCurrent = indexerMotor.getSupplyCurrent();

    conveyorRollerVelocity = conveyorMotor.getVelocity();
    conveyorRollerAppliedVolts = conveyorMotor.getMotorVoltage();
    conveyorRollerCurrent = conveyorMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftRollerVelocity,
        leftRollerAppliedVolts,
        leftRollerCurrent,
        rightRollerVelocity,
        rightRollerAppliedVolts,
        rightRollerCurrent,
        indexerRollerVelocity,
        indexerRollerAppliedVolts,
        indexerRollerCurrent,
        conveyorRollerVelocity,
        conveyorRollerAppliedVolts,
        conveyorRollerCurrent);

    leftRollerMotor.optimizeBusUtilization();
    rightRollerMotor.optimizeBusUtilization();
    indexerMotor.optimizeBusUtilization();
    conveyorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.leftRollerConnected =
        BaseStatusSignal.isAllGood(leftRollerVelocity, leftRollerAppliedVolts, leftRollerCurrent);

    inputs.leftRollerSpeed = leftRollerVelocity.getValue();
    inputs.leftRollerAppliedVolts = leftRollerAppliedVolts.getValue();
    inputs.leftRollerCurrent = leftRollerCurrent.getValue();

    inputs.rightRollerConnected =
        BaseStatusSignal.isAllGood(
            rightRollerVelocity, rightRollerAppliedVolts, rightRollerCurrent);
    inputs.rightRollerSpeed = rightRollerVelocity.getValue();
    inputs.rightRollerAppliedVolts = rightRollerAppliedVolts.getValue();
    inputs.rightRollerCurrent = rightRollerCurrent.getValue();

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
  public void setSideRollersSpeed(AngularVelocity speed) {
    leftRollerMotor.setControl(velocityReq.withVelocity(speed));
    rightRollerMotor.setControl(velocityReq.withVelocity(speed));
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
  public void stopSideRollers() {
    leftRollerMotor.setControl(neutralOut);
    rightRollerMotor.setControl(neutralOut);
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
