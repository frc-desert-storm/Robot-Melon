package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setState(State state) {
    this.state = state;
    switch (state) {
      case SHOOTING -> {
        io.setIndexerSpeed(RPM.of(120));
      }
      case REVERSE -> {
        io.setIndexerSpeed(RPM.of(-40));
      }
      case IDLE -> {
        io.stopIndexer();
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public void stop() {
    state = State.IDLE;
    io.stopIndexer();
  }

  public State state = State.IDLE;

  public enum State {
    SHOOTING,
    REVERSE,
    IDLE
  }
}
