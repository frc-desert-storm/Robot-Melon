package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the over-the-bumper intake mechanism.
 *
 * <h3>Mechanism overview</h3>
 *
 * <ul>
 *   <li><b>Lift arm</b> – raises and lowers the intake over the bumper using MotionMagic position
 *       control.
 *   <li><b>Rollers</b> – spins intake rollers (leader-follower) to pull fuel into the robot.
 * </ul>
 *
 * <h3>Configurable setpoints (mechanism rotations, post gear-ratio)</h3>
 *
 * <ul>
 *   <li>{@link #LIFT_DOWN_POSITION_ROT} – arm fully deployed for intaking
 *   <li>{@link #LIFT_UP_POSITION_ROT} – arm stowed above the bumper
 * </ul>
 */
public class Intake extends SubsystemBase {

  // ── Lift setpoints (mechanism rotations) ─────────────────────────────────
  /** Arm fully lowered – intake is outside the robot perimeter for ground pickup. */
  public static final double LIFT_DOWN_POSITION_ROT = -12.0;

  /** Arm fully raised – intake is stowed inside the frame perimeter. */
  public static final double LIFT_UP_POSITION_ROT = 0.0;

  // ── Roller voltages ─────────────────────────────────────────
  /** Voltage applied to rollers during normal intaking. */
  public static final double ROLLER_INTAKE_VOLTS = 10.0;

  /** Voltage applied to rollers to eject game pieces. */
  public static final double ROLLER_EJECT_VOLTS = -10.0;

  // ── IO layer ─────────────────────────────────────────────────────────────
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // ── Cached state ──────────────────────────────────────────────────────────
  private boolean liftLowered = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  // =========================================================================
  // Periodic
  // =========================================================================

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/LiftLowered", liftLowered);
  }

  // =========================================================================
  // Low-level API (called by commands or the composite subsystem)
  // =========================================================================

  /** Command the lift arm to the deployed (lowered) position. */
  public void lowerIntake() {
    liftLowered = true;
    io.setLiftPosition(LIFT_DOWN_POSITION_ROT);
  }

  /** Command the lift arm to the stowed (raised) position. */
  public void raiseIntake() {
    liftLowered = false;
    io.setLiftPosition(LIFT_UP_POSITION_ROT);
  }

  /** Spin rollers in the intaking direction. */
  public void runRollersForward() {
    io.setRollerVoltage(ROLLER_INTAKE_VOLTS);
  }

  /** Spin rollers in the ejecting direction. */
  public void runRollersReverse() {
    io.setRollerVoltage(ROLLER_EJECT_VOLTS);
  }

  /** Stop all roller motors. */
  public void stopRollers() {
    io.setRollerVoltage(0.0);
  }

  /** Override the lift with a direct voltage (open-loop). */
  public void setLiftVoltage(double volts) {
    io.setLiftVoltage(volts);
  }

  /** Zero the lift encoder at the current position (call at a known hard-stop). */
  public void resetLiftEncoder() {
    io.resetLiftEncoder();
  }

  // =========================================================================
  // Getters
  // =========================================================================

  public double getLiftPositionRot() {
    return inputs.liftPositionRot;
  }

  public double getRollerVelocityRpm() {
    return inputs.rollerVelocityRpm;
  }

  public boolean isLiftLowered() {
    return liftLowered;
  }

  /**
   * Returns {@code true} when the lift arm is within tolerance of its target.
   *
   * @param toleranceRot Allowable error in mechanism rotations.
   */
  public boolean isLiftAtTarget(double toleranceRot) {
    double target = liftLowered ? LIFT_DOWN_POSITION_ROT : LIFT_UP_POSITION_ROT;
    return Math.abs(inputs.liftPositionRot - target) < toleranceRot;
  }

  // =========================================================================
  // Pre-built commands (public for use outside this subsystem)
  // =========================================================================

  /** Lower the intake arm and hold position until interrupted. */
  public Command lowerCommand() {
    return Commands.startEnd(this::lowerIntake, () -> {}, this);
  }

  /** Raise the intake arm and hold position until interrupted. */
  public Command raiseCommand() {
    return Commands.startEnd(this::raiseIntake, () -> {}, this);
  }

  /** Run rollers forward until interrupted, then stop. */
  public Command rollersForwardCommand() {
    return Commands.startEnd(this::runRollersForward, this::stopRollers, this);
  }

  /** Run rollers in reverse until interrupted, then stop. */
  public Command rollersReverseCommand() {
    return Commands.startEnd(this::runRollersReverse, this::stopRollers, this);
  }

  /**
   * Lower the intake <em>and then</em> immediately run the rollers. When interrupted the rollers
   * stop but the arm stays at its last position.
   */
  public Command deployAndIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(this::lowerIntake, this),
        Commands.waitUntil(() -> isLiftAtTarget(0.5)),
        Commands.startEnd(this::runRollersForward, this::stopRollers, this));
  }
}
