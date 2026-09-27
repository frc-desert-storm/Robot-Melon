package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IndexerConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
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
        io.setIndexerVolts(Volts.of(12.0));
        startedShootingTime = Timer.getFPGATimestamp();
      }
      case REVERSE -> {
        io.setIndexerVolts(Volts.of(-12.0));
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
        //        io.setIndexerSpeed(commandedSpeed);

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
    update3dPose(inputs.indexerRollerAngle);
  }

  public void stop() {
    state = State.IDLE;
    io.stopIndexer();
  }

  public void update3dPose(Angle azimuthAngle) {
    Pose3d indexerPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, -azimuthAngle.in(Radians)));
    Logger.recordOutput("Indexer/IndexerPose", indexerPose);
  }

  public State state = State.IDLE;

  public enum State {
    SHOOTING,
    REVERSE,
    IDLE
  }
}
