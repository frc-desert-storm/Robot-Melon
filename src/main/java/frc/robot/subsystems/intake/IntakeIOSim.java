package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation implementation of {@link IntakeIO}.
 *
 * <p>Uses WPILib {@link DCMotorSim} to model each motor group.
 */
public class IntakeIOSim implements IntakeIO {

  // ── Simulation plants ─────────────────────────────────────────────────────
  private final DCMotorSim liftSim;
  private final DCMotorSim rollerSim;

  // ── Simulated state ───────────────────────────────────────────────────────
  private double liftAppliedVolts = 0.0;
  private double rollerAppliedVolts = 0.0;

  /** Simple position-control PID used in sim to mimic MotionMagic. */
  private final PIDController liftSimPid = new PIDController(20.0, 0.0, 0.5);

  private boolean usePositionControl = false;
  private double liftTargetRot = 0.0;

  private static final double LOOP_PERIOD_SECS = 0.02;

  // Assumed moments of inertia (kg·m²) – tune for realistic feel
  private static final double LIFT_MOI = 0.005;
  private static final double ROLLER_MOI = 0.001;

  public IntakeIOSim() {
    liftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), LIFT_MOI, IntakeIOKraken.LIFT_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));

    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), ROLLER_MOI, IntakeIOKraken.ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Resolve lift voltage (position-control PID if active, else open-loop)
    if (usePositionControl) {
      liftAppliedVolts =
          MathUtil.clamp(
              liftSimPid.calculate(liftSim.getAngularPositionRotations(), liftTargetRot),
              -12.0,
              12.0);
    }

    liftSim.setInputVoltage(MathUtil.clamp(liftAppliedVolts, -12.0, 12.0));
    rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));

    liftSim.update(LOOP_PERIOD_SECS);
    rollerSim.update(LOOP_PERIOD_SECS);

    // ── Lift ──────────────────────────────────────────────────────────────
    inputs.liftMotorConnected = true;
    inputs.liftPositionRot = liftSim.getAngularPositionRotations();
    inputs.liftVelocityRpm = liftSim.getAngularVelocityRPM();
    inputs.liftAppliedVolts = liftAppliedVolts;
    inputs.liftCurrentAmps = Math.abs(liftSim.getCurrentDrawAmps());
    inputs.liftTempCelsius = 25.0;

    // ── Roller ───────────────────────
    inputs.rollerMotorConnected = true;
    inputs.rollerVelocityRpm = rollerSim.getAngularVelocityRPM();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
    inputs.rollerTempCelsius = 25.0;
  }

  @Override
  public void setLiftPosition(double positionRot) {
    usePositionControl = true;
    liftTargetRot = positionRot;
  }

  @Override
  public void setLiftVoltage(double volts) {
    usePositionControl = false;
    liftAppliedVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  @Override
  public void resetLiftEncoder() {
    // DCMotorSim does not expose a direct reset; mark position as 0 via PID target reset
    liftTargetRot = 0.0;
  }

  @Override
  public void setLiftBrakeMode(boolean brake) {
    // No-op in sim
  }
}
