package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  // ---- Constants ----
  public static final double VELOCITY_TOLERANCE_RPM = 75.0;
  public static final double READY_TIME_SEC = 0.15;
  public static final double HARDSTOP_CURRENT_AMPS = 70.0;
  public static final double JAM_CURRENT_AMPS = 80.0;

  public static final double GEARED_INERTIA = 0.005;
  public static final double MOTOR_REVOLUTIONS_PER_RADIAN = 1.0;
  public static final double GEAR_RATIO = 28.125;

  @AutoLog
  public static class TurretIOInputs {
    public double velocityRPM = 0.0;
    public double velocitySetpointRPM = 0.0;

    public double currentAngle = 0.0;
    public double angleSetpointDegrees = 0.0;

    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;

    public double motorTempCelsius = 0.0;

    public boolean atSetpoint = false;
    public boolean ready = false;
    public boolean hardstopDetected = false;
    public boolean goingRight = false;
    public boolean jamDetected = false;
  }

  void updateInputs(TurretIOInputs inputs);

  // void rotateRight(double rpm);

  // void rotateLeft(double rpm);

  // void rotateToTargetpoint(double degrees);

  void setCurrentAngleDegrees(double degrees);

  void setVelocityRPM(double rpm); 

  void setVoltage(double volts);

  void stop();
}
