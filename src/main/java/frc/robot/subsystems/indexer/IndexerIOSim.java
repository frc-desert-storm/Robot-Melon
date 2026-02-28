package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation implementation of {@link IndexerIO}.
 *
 * <p>Models the leader motor; the follower is represented by mirroring the leader's current draw
 * (since in hardware it physically runs in the opposite direction but at the same commanded power).
 */
public class IndexerIOSim implements IndexerIO {

  private static final double LOOP_PERIOD_SECS = 0.02;
  private static final double INDEXER_MOI = 0.001; // kg·m²

  private final DCMotorSim leaderSim;
  private double appliedVolts = 0.0;

  public IndexerIOSim() {
    leaderSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), INDEXER_MOI, 1.0),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    leaderSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    leaderSim.update(LOOP_PERIOD_SECS);

    inputs.leaderMotorConnected = true;
    inputs.followerMotorConnected = true;

    inputs.leaderVelocityRpm = leaderSim.getAngularVelocityRPM();
    inputs.leaderAppliedVolts = appliedVolts;
    inputs.leaderCurrentAmps = Math.abs(leaderSim.getCurrentDrawAmps());
    inputs.leaderTempCelsius = 25.0;

    // Follower mirrors current (inverted in hardware, same load in sim)
    inputs.followerCurrentAmps = Math.abs(leaderSim.getCurrentDrawAmps());
    inputs.followerTempCelsius = 25.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
