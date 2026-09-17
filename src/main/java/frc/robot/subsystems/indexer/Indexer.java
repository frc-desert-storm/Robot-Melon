package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IndexerConstants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Supplier<Distance> distanceSupplier;

  private Double stalledTime = 0.0;
  private Double startedShootingTime = 0.0;

  public Indexer(IndexerIO io, Supplier<Distance> distanceSupplier) {
    this.io = io;
    this.distanceSupplier = distanceSupplier;
  }

  public void setState(State state) {
    this.state = state;
    switch (state) {
      case SHOOTING -> {
        var commandedSpeed = RPM.of(INDEXER_SPEED_MAP.get(distanceSupplier.get().in(Meters)));
        io.setIndexerSpeed(commandedSpeed);
        startedShootingTime = Timer.getFPGATimestamp();
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

    Distance distance = distanceSupplier.get();
    Logger.recordOutput("Indexer/DistanceToTarget", distance == null ? 0.0 : distance.in(Meters));

    switch (state) {
      case SHOOTING -> {
        var commandedSpeed = RPM.of(INDEXER_SPEED_MAP.get(distanceSupplier.get().in(Meters)));
        io.setIndexerSpeed(commandedSpeed);

        if (inputs.indexerRollerSpeed.abs(RotationsPerSecond) <= 0.02
            && startedShootingTime < Timer.getFPGATimestamp() - 1) {
          setState(State.REVERSE);
          stalledTime = Timer.getFPGATimestamp();
        }
      }
      case REVERSE -> {
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
