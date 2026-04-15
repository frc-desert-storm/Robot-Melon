// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean turnMotorConnected;
    public Voltage turnAppliedVolts;
    public Current turnCurrent;
    public Current turnSupplyCurrent;
    public Angle turnPosition;
    public Angle turnSetpoint;
    public AngularVelocity turnVelocity;

    public boolean hoodMotorConnected;
    public Voltage hoodAppliedVolts;
    public Current hoodCurrent;
    public Current hoodSupplyCurrent;
    public Angle hoodPosition;
    public Angle hoodSetpoint;
    public AngularVelocity hoodVelocity;

    public boolean flywheelMotorConnected;
    public Voltage flywheelAppliedVolts;
    public Current flywheelCurrent;
    public Current flywheelSupplyCurrent;
    public AngularVelocity flywheelSpeed;
    public AngularAcceleration flywheelAccel;
    public AngularVelocity flywheelSetpointSpeed;
    public AngularAcceleration flywheelSetpointAccel;
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
