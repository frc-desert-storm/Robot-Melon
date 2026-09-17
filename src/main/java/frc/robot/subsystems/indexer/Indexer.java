package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private Double stalledTime = 0.0;
  private Double startedShootingTime = 0.0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setState(State state) {
    this.state = state;
    switch (state) {
      case SHOOTING -> {
        //        io.setIndexerSpeed(RPM.of(120));
        startedShootingTime = Timer.getFPGATimestamp();
        io.setIndexerVolts(Volts.of(12));
      }
      case REVERSE -> {
        io.setIndexerSpeed(RPM.of(-40));
      }
      case IDLE -> {
        io.stopIndexer();
        // io.setIndexerSpeed(RPM.of(-20));
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    switch (state) {
      case SHOOTING -> {
        //        io.setIndexerVolts(Volts.of(12));
        if (inputs.indexerRollerSpeed.abs(RotationsPerSecond) <= 0.02
            && startedShootingTime < Timer.getFPGATimestamp() - 1) {
          setState(State.REVERSE);
          stalledTime = Timer.getFPGATimestamp();
        }
      }
      case REVERSE -> {
        //        io.setIndexerSpeed(RPM.of(-40));
        if (stalledTime < Timer.getFPGATimestamp() - 1) {
          setState(State.SHOOTING);
        }
      }
      case IDLE -> {
        io.stopIndexer();
      }
    }
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
