package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Timer readyTimer = new Timer();

  private final SysIdRoutine sysIdRoutine;

  public Shooter(ShooterIO io) {
    this.io = io;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setVoltage(volts.in(edu.wpi.first.units.Units.Volts)), null, this));
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);

    if (inputs.atSetpoint) {
      if (!readyTimer.isRunning()) {
        readyTimer.restart();
      }
    } else {
      readyTimer.stop();
      readyTimer.reset();
    }

    inputs.ready = readyTimer.hasElapsed(ShooterIO.READY_TIME_SEC);

    Logger.processInputs("Shooter", inputs);
  }

  public void setRPM(double rpm) {
    io.setVelocityRPM(rpm);
  }

  public void stop() {
    io.stop();
  }

  public boolean isReady() {
    return inputs.ready;
  }

  public boolean jamDetected() {
    return inputs.jamDetected;
  }

  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }
}
