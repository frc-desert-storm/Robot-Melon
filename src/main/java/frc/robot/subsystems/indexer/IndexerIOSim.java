package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOSim implements IndexerIO {
  private AngularVelocity rollerSetpoint = RPM.of(0);
  private Voltage appliedVolts = Volts.of(0);
  private Angle rollerAngle = Radians.of(0);

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerRollerConnected = true;
    rollerAngle = rollerAngle.plus(rollerSetpoint.times(Seconds.of(0.02)));

    inputs.indexerRollerSpeed = rollerSetpoint;
    inputs.indexerRollerAngle = rollerAngle;
    inputs.indexerRollerAppliedVolts = appliedVolts;
    inputs.indexerRollerCurrent = Amps.of(0.0);
  }

  @Override
  public void setIndexerSpeed(AngularVelocity speed) {
    rollerSetpoint = speed;
    appliedVolts = Volts.of(speed.in(RotationsPerSecond) * 4.9);
  }

  @Override
  public void setIndexerVolts(Voltage volts) {
    appliedVolts = volts;
    rollerSetpoint = RotationsPerSecond.of(volts.in(Volts) / 4.9);
  }

  @Override
  public void stopIndexer() {
    rollerSetpoint = RPM.of(0);
    appliedVolts = Volts.of(0.0);
  }
}
