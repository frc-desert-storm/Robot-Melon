package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the over-the-bumper intake subsystem.
 *
 * <p>Motor layout:
 *
 * <ul>
 *   <li>Motor 1 – Lift arm (raise / lower the intake over the bumper)
 *   <li>Motor 2 – Roller leader (spins the intake rollers)
 * </ul>
 *
 * <p>Motors 2 and 3 are wired in a leader-follower configuration so a single voltage command drives
 * both.
 */
public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    // ── Lift motor ──────────────────────────────────────────────────────────
    public boolean liftMotorConnected = false;
    public double liftPositionRot = 0.0; // mechanism rotations (post gear-ratio)
    public double liftVelocityRpm = 0.0;
    public double liftAppliedVolts = 0.0;
    public double liftCurrentAmps = 0.0;
    public double liftTempCelsius = 0.0;

    // ── Roller ──────────────────────────────────────────────────────────────
    public boolean rollerMotorConnected = false;
    public double rollerVelocityRpm = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerTempCelsius = 0.0;
  }

  /** Update logged inputs. Called every robot loop. */
  default void updateInputs(IntakeIOInputs inputs) {}

  // ── Lift control ─────────────────────────────────────────────────────────

  /**
   * Command the lift motor to a target position using MotionMagic.
   *
   * @param positionRot Target position in mechanism rotations (post gear-ratio).
   */
  default void setLiftPosition(double positionRot) {}

  /**
   * Open-loop voltage control for the lift motor (used for tuning / override).
   *
   * @param volts Voltage to apply, positive = raise.
   */
  default void setLiftVoltage(double volts) {}

  // ── Roller control ─────────────────────────────────────────────

  /**
   * Set the roller leader voltage
   *
   * @param volts Voltage to apply, positive = intake direction.
   */
  default void setRollerVoltage(double volts) {}

  /** Zero the lift encoder. Call after the arm reaches a known hard-stop. */
  default void resetLiftEncoder() {}

  /** Enable or disable motor brake mode for the lift. */
  default void setLiftBrakeMode(boolean brake) {}
}
