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
    public Voltage leftRollerAppliedVolts;
    public Current leftRollerCurrent;
    public AngularVelocity leftRollerSpeed;

    public boolean rightRollerConnected;
    public Voltage rightRollerAppliedVolts;
    public Current rightRollerCurrent;
    public AngularVelocity rightRollerSpeed;

    public boolean indexerRollerConnected;
    public Voltage indexerRollerAppliedVolts;
    public Current indexerRollerCurrent;
    public AngularVelocity indexerRollerSpeed;

    public boolean conveyorRollerConnected;
    public Voltage conveyorRollerAppliedVolts;
    public Current conveyorRollerCurrent;
    public AngularVelocity conveyorRollerSpeed;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setSideRollersSpeed(AngularVelocity speed) {}

  default void setIndexerSpeed(AngularVelocity speed) {}

  default void setConveyorSpeed(AngularVelocity speed) {}

  default void stopSideRollers() {}

  default void stopIndexer() {}

  default void stopConveyor() {}
}
