// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean turnMotorConnected = false;
    public Voltage turnAppliedVolts = Volts.of(0.0);
    public Current turnCurrent = Amps.of(0.0);
    public Current turnSupplyCurrent = Amps.of(0.0);
    public Angle turnPosition = Degrees.of(0.0);
    public Angle turnSetpoint = Degrees.of(0.0);
    public AngularVelocity turnVelocity = RPM.of(0.0);

    public boolean hoodMotorConnected = false;
    public Voltage hoodAppliedVolts = Volts.of(0.0);
    public Current hoodCurrent = Amps.of(0.0);
    public Current hoodSupplyCurrent = Amps.of(0.0);
    public Angle hoodPosition = Degrees.of(0.0);
    public Angle hoodSetpoint = Degrees.of(0.0);
    public AngularVelocity hoodVelocity = RPM.of(0.0);

    public boolean flywheelMotorConnected = false;
    public Voltage flywheelAppliedVolts = Volts.of(0.0);
    public Current flywheelCurrent = Amps.of(0.0);
    public Current flywheelSupplyCurrent = Amps.of(0.0);
    public AngularVelocity flywheelSpeed = RPM.of(0.0);
    public AngularAcceleration flywheelAccel = RotationsPerSecondPerSecond.of(0.0);
    public AngularVelocity flywheelSetpointSpeed = RPM.of(0.0);
    public AngularAcceleration flywheelSetpointAccel = RotationsPerSecondPerSecond.of(0.0);
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTurnSetpoint(Angle position, AngularVelocity velocity) {}

  public default void setHoodAngle(Angle angle) {}

  public default void setHoodOut(Voltage out) {}

  public default void setFlywheelSpeed(AngularVelocity speed) {}

  public default void stopTurn() {}

  public default void stopHood() {}

  public default void stopFlywheel() {}

  public default void resetTurnEncoder() {}

  public default void zeroHoodPosition() {}

  public default void setTurnPID(double kP, double kD, double kV, double kS) {}

  public default void setHoodPID(double kP, double kD, double kS) {}

  public default void setFlywheelPID(double kP, double kD, double kV, double kS) {}
}
