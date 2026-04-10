package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  class IndexerIOInputs {
    public boolean indexerRollerConnected;
    public Voltage indexerRollerAppliedVolts = Volts.of(0.0);
    public Current indexerRollerCurrent = Amps.of(0.0);
    public AngularVelocity indexerRollerSpeed = RadiansPerSecond.of(0.0);

    public boolean conveyorRollerConnected;
    public Voltage conveyorRollerAppliedVolts = Volts.of(0.0);
    public Current conveyorRollerCurrent = Amps.of(0.0);
    public AngularVelocity conveyorRollerSpeed = RadiansPerSecond.of(0.0);
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setIndexerSpeed(AngularVelocity speed) {}

  default void setConveyorSpeed(AngularVelocity speed) {}

  default void stopIndexer() {}

  default void stopConveyor() {}
}
