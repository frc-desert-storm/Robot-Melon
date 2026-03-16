package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  class IndexerIOInputs {
    public boolean leftRollerConnected;
    public Voltage leftRollerAppliedVolts = Volts.of(0.0);
    public Current leftRollerCurrent = Amps.of(0.0);
    public AngularVelocity leftRollerSpeed = RadiansPerSecond.of(0.0);

    public boolean rightRollerConnected;
    public Voltage rightRollerAppliedVolts = Volts.of(0.0);
    public Current rightRollerCurrent = Amps.of(0.0);
    public AngularVelocity rightRollerSpeed = RadiansPerSecond.of(0.0);

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

  default void setSideRollersSpeed(AngularVelocity speed) {}

  default void setIndexerSpeed(AngularVelocity speed) {}

  default void setConveyorSpeed(AngularVelocity speed) {}

  default void stopSideRollers() {}

  default void stopIndexer() {}

  default void stopConveyor() {}
}
