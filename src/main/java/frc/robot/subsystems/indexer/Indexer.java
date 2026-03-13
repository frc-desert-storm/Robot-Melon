package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public Command setState(State state) {
    return runOnce(
        () -> {
          this.state = state;
          switch (state) {
            case SCORING -> {
              io.setSideRollersSpeed(RotationsPerSecond.of(3000.0 / 60));
              io.setConveyorSpeed(RotationsPerSecond.of(2000.0 / 60));
              io.setIndexerSpeed(RotationsPerSecond.of(2600.0 / 60));
            }
            case FEEDING -> {
              io.setSideRollersSpeed(RotationsPerSecond.of(2000.0 / 60));
              io.setConveyorSpeed(RotationsPerSecond.of(2000.0 / 60));
            }
            case IDLE -> {
              io.stopIndexer();
              io.stopSideRollers();
              io.stopConveyor();
            }
          }
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public void stop() {
    state = State.IDLE;
    io.stopIndexer();
    io.stopSideRollers();
    io.stopConveyor();
  }

  public State state = State.IDLE;

  public enum State {
    SCORING,
    FEEDING,
    IDLE
  }
}
