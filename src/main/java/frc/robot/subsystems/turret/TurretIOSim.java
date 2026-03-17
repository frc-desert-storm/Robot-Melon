// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.intake.Intake;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class TurretIOSim implements TurretIO {
  private Intake intake;
  private SwerveDriveSimulation driveSimulation;
  private Angle turnPosition = Radians.zero();
  private Angle hoodAngle = Radians.zero();
  private AngularVelocity flywheelGoal = RadiansPerSecond.zero();

  private SlewRateLimiter flywheelAccelLimiter =
      new SlewRateLimiter(RPM.of(4000).div(Seconds.of(1)).in(RadiansPerSecondPerSecond));

  public TurretIOSim(Intake intake, SwerveDriveSimulation driveSimulation) {
    this.intake = intake;
    this.driveSimulation = driveSimulation;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turnPosition = turnPosition;
    inputs.hoodPosition = hoodAngle;
    inputs.flywheelSpeed =
        RadiansPerSecond.of(flywheelAccelLimiter.calculate(flywheelGoal.in(RadiansPerSecond)));
  }

  @Override
  public void setTurnSetpoint(Angle position, AngularVelocity velocity) {
    turnPosition = position;
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodAngle = angle;
  }

  @Override
  public void setFlywheelSpeed(AngularVelocity speed) {
    flywheelGoal = speed;
  }

  @Override
  public void stopFlywheel() {
    flywheelGoal = RadiansPerSecond.zero();
  }
}
