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

  // Voltage constants for spinning and unjamming the indexer
  public static final double FEED_VOLTS = 5.0;
  public static final double REVERSE_VOLTS = -2.0;

  //
  public static final double FEED_RPM = 4000;
  public static final double REVERSE_RPM = -3000;

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
    io.setSideRollersVelocityRPM(FEED_RPM);
  }

  /** Run the indexer in reverse (eject / un-jam). */
  public void runReverse() {
    io.setSideRollersVelocityRPM(-FEED_RPM);
  }

  /** Stop the indexer. */
  public void stopSideRollers() {
    io.setSideRollerVoltage(0);
  }

  public void stopIndexer() {
    io.setIndexerVoltage(0);
  }

  // =========================================================================
  // Getters
  // =========================================================================

  public double getVelocityRpm() {
    return inputs.leaderVelocityRpm;
  }

  public void runIndexerForward() {
    io.setIndexerRPM(FEED_RPM);
  }

  public void runBoth() {
    io.setIndexerRPM(FEED_RPM);
    io.setSideRollersVelocityRPM(FEED_RPM);
  }

  public void stop() {
    io.setSideRollerVoltage(0);
    io.setIndexerVoltage(0);
  }

  // =========================================================================
  // Pre-built commands
  // =========================================================================

  /** Run the indexer forward until interrupted, then stop. */
  public Command forwardCommand() {
    return Commands.startEnd(this::runForward, this::stopSideRollers, this);
  }

  /** Run the indexer in reverse until interrupted, then stop. */
  public Command reverseCommand() {
    return Commands.startEnd(this::runReverse, this::stopSideRollers, this);
  }

  public Command loadShooter() {
    return Commands.startEnd(this::runIndexerForward, this::stopSideRollers, this);
  }

  public Command indexConveyor() {
    return Commands.startEnd(this::runBoth, this::stop, this);
  }
}
