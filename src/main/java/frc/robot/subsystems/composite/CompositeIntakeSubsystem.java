package frc.robot.subsystems.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

/**
 * Composite subsystem that coordinates the {@link Intake} and {@link Indexer} to work in tandem.
 *
 * <p>This class does <em>not</em> extend SubsystemBase itself; it wraps both child subsystems
 * (which are already registered with the scheduler) and exposes higher-level commands that
 * orchestrate them together.
 *
 * <h3>Composite modes</h3>
 *
 * <ul>
 *   <li><b>Forward</b> – lowers the intake, runs rollers + conveyor forward, and simultaneously
 *       runs the indexer forward to pass fuel to the shooter.
 *   <li><b>Reverse</b> – runs rollers + conveyor and indexer in reverse to eject game pieces out of
 *       the robot.
 * </ul>
 *
 * <p>Individual subsystem commands (lift, rollers-only, indexer-only) are also accessible through
 * this class so that RobotContainer only needs to hold a reference to the composite.
 */
public class CompositeIntakeSubsystem extends SubsystemBase {

  private final Intake intake;
  private final Indexer indexer;

  /**
   * Create a new CompositeIntakeSubsystem.
   *
   * @param intake the already-constructed {@link Intake}
   * @param indexer the already-constructed {@link Indexer}
   */
  public CompositeIntakeSubsystem(Intake intake, Indexer indexer) {
    this.intake = intake;
    this.indexer = indexer;
  }

  // =========================================================================
  // Periodic – log composite state
  // =========================================================================

  @Override
  public void periodic() {
    Logger.recordOutput("CompositeIntake/IntakeLiftLowered", intake.isLiftLowered());
    Logger.recordOutput("CompositeIntake/IntakeRollerVelocityRpm", intake.getRollerVelocityRpm());
    Logger.recordOutput("CompositeIntake/IndexerVelocityRpm", indexer.getVelocityRpm());
  }

  // =========================================================================
  // Composite commands
  // =========================================================================

  /**
   * <b>Composite Forward</b> – lowers the intake arm, then runs the intake rollers/conveyor AND the
   * indexer in the forward direction simultaneously.
   *
   * <p>When interrupted: rollers and indexer stop, but the arm holds its last position (caller must
   * issue a raise command separately if desired).
   *
   * @return a command requiring both child subsystems
   */
  public Command compositeForwardCommand() {
    return Commands.sequence(
        // Step 1: Deploy the arm (does not require the indexer subsystem)
        Commands.runOnce(intake::lowerIntake, intake),
        // Step 2: Wait for the arm to reach the deployed position
        Commands.waitUntil(() -> intake.isLiftAtTarget(0.75)),
        // Step 3: Run intake rollers + indexer in parallel until interrupted
        Commands.parallel(
            Commands.startEnd(intake::runRollersForward, intake::stopRollers, intake),
            Commands.startEnd(indexer::runForward, indexer::stop, indexer)));
  }

  /**
   * <b>Composite Reverse</b> – runs intake rollers/conveyor AND the indexer in reverse
   * simultaneously (eject/un-jam). The arm stays at its current position.
   *
   * @return a command requiring both child subsystems
   */
  public Command compositeReverseCommand() {
    return Commands.parallel(
        Commands.startEnd(intake::runRollersReverse, intake::stopRollers, intake),
        Commands.startEnd(indexer::runReverse, indexer::stop, indexer));
  }

  // =========================================================================
  // Individual intake commands (delegated)
  // =========================================================================

  /**
   * Lower (deploy) the intake arm.
   *
   * @return command requiring the intake subsystem
   */
  public Command intakeLowerCommand() {
    return intake.lowerCommand();
  }

  /**
   * Raise (stow) the intake arm.
   *
   * @return command requiring the intake subsystem
   */
  public Command intakeRaiseCommand() {
    return intake.raiseCommand();
  }

  /**
   * Run intake rollers and conveyor forward only (indexer unaffected).
   *
   * @return command requiring the intake subsystem
   */
  public Command intakeForwardCommand() {
    return intake.rollersForwardCommand();
  }

  /**
   * Run intake rollers and conveyor in reverse only (indexer unaffected).
   *
   * @return command requiring the intake subsystem
   */
  public Command intakeReverseCommand() {
    return intake.rollersReverseCommand();
  }

  // =========================================================================
  // Individual indexer commands (delegated)
  // =========================================================================

  /**
   * Run the indexer forward only (intake unaffected).
   *
   * @return command requiring the indexer subsystem
   */
  public Command indexerForwardCommand() {
    return indexer.forwardCommand();
  }

  /**
   * Run the indexer in reverse only (intake unaffected).
   *
   * @return command requiring the indexer subsystem
   */
  public Command indexerReverseCommand() {
    return indexer.reverseCommand();
  }

  // =========================================================================
  // Getters
  // =========================================================================

  public Intake getIntake() {
    return intake;
  }

  public Indexer getIndexer() {
    return indexer;
  }
}
