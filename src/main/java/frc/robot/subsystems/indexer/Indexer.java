package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the indexer mechanism feeding fuel into the shooter.
 *
 * <p>Two Kraken X60 motors run in a leader-follower configuration with the follower's direction
 * inverted so both sides of the feed path push in the same linear direction.
 */
public class Indexer extends SubsystemBase {

  // ── Voltage constants ────────────────────────────────────────────────────
  /** Voltage applied while feeding fuel toward the shooter. */
  public static final double FEED_VOLTS = 10.0;

  /** Voltage applied while reversing the indexer (e.g., un-jamming). */
  public static final double REVERSE_VOLTS = -10.0;

  // ── IO layer ─────────────────────────────────────────────────────────────
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  // =========================================================================
  // Periodic
  // =========================================================================

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  // =========================================================================
  // Low-level API
  // =========================================================================

  /** Run the indexer in the forward (feed-to-shooter) direction. */
  public void runForward() {
    io.setVoltage(FEED_VOLTS);
  }

  /** Run the indexer in reverse (eject / un-jam). */
  public void runReverse() {
    io.setVoltage(REVERSE_VOLTS);
  }

  /** Stop the indexer. */
  public void stop() {
    io.setVoltage(0.0);
  }

  // =========================================================================
  // Getters
  // =========================================================================

  public double getVelocityRpm() {
    return inputs.leaderVelocityRpm;
  }

  // =========================================================================
  // Pre-built commands
  // =========================================================================

  /** Run the indexer forward until interrupted, then stop. */
  public Command forwardCommand() {
    return Commands.startEnd(this::runForward, this::stop, this);
  }

  /** Run the indexer in reverse until interrupted, then stop. */
  public Command reverseCommand() {
    return Commands.startEnd(this::runReverse, this::stop, this);
  }
}
