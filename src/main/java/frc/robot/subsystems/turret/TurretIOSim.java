// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretIOSim implements TurretIO {
  private Angle turnSetpoint = Degrees.of(0);
  private AngularVelocity turnVelocityCmd = RadiansPerSecond.of(0);
  private Angle hoodSetpoint = Degrees.of(12);
  private AngularVelocity flywheelSetpoint = RPM.of(0);

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turnMotorConnected = true;
    inputs.hoodMotorConnected = true;
    inputs.flywheelMotorConnected = true;
    inputs.flywheelFollowerMotorConnected = true;

    inputs.turnPosition = turnSetpoint;
    inputs.turnSetpoint = turnSetpoint;
    inputs.turnVelocity = turnVelocityCmd;

    inputs.hoodPosition = hoodSetpoint;
    inputs.hoodSetpoint = hoodSetpoint;
    inputs.hoodVelocity = RadiansPerSecond.of(0);

    inputs.flywheelSpeed = flywheelSetpoint;
    inputs.flywheelFollowerSpeed = flywheelSetpoint;
    inputs.flywheelSetpointSpeed = flywheelSetpoint;
  }

  @Override
  public void setTurnSetpoint(Angle position, AngularVelocity velocity) {
    turnSetpoint = position;
    turnVelocityCmd = velocity;
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodSetpoint = angle;
  }

  @Override
  public void setFlywheelSpeed(AngularVelocity speed) {
    flywheelSetpoint = speed;
  }

  @Override
  public void stopTurn() {
    turnVelocityCmd = RadiansPerSecond.of(0);
  }

  @Override
  public void stopFlywheel() {
    flywheelSetpoint = RPM.of(0);
  }
}
