package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  // ---- Constants ----
  public static final double VELOCITY_TOLERANCE_RPM = 75.0;
  public static final double READY_TIME_SEC = 0.15;
  public static final double JAM_CURRENT_AMPS = 80.0;

  public static final double gearedInertia = 0.005;
  public static final double motorRevolutionsPerRadian = 1.0;
  public static final double geatRatio = 1.0;

  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRPM = 0.0;
    public double velocitySetpointRPM = 0.0;

    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;

    public double motorTempCelsius = 0.0;

    public boolean atSetpoint = false;
    public boolean ready = false;
    public boolean jamDetected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVelocityRPM(double rpm) {}

  default void setVoltage(double volts) {}

  default void stop() {}
}
