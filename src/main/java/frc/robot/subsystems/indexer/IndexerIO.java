package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the indexer subsystem.
 *
 * <p>The indexer uses two Kraken X60 motors in a leader-follower configuration. The follower runs
 * in the opposite direction so that both motors push the fuel toward the shooter together (e.g.,
 * one motor on each side of a feeding path).
 */
public interface IndexerIO {

  @AutoLog
  class IndexerIOInputs {
    public boolean leaderMotorConnected = false;
    public boolean followerMotorConnected = false;

    // Leader signals (follower mirrors these via hardware follower mode)
    public double leaderVelocityRpm = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    // Follower monitoring
    public double followerCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  /** Update logged inputs. Called every robot loop. */
  default void updateInputs(IndexerIOInputs inputs) {}

  /**
   * Set the indexer leader voltage; the follower (direction-inverted) mirrors it.
   *
   * @param volts Voltage to apply, positive = toward-shooter direction.
   */
  default void setVoltage(double volts) {}
}
