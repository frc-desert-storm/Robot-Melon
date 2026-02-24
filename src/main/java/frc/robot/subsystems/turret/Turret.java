package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

// import java.io.Console;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Timer readyTimer = new Timer();

  private final SysIdRoutine sysIdRoutine;

  public Turret(TurretIO io) {
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

    inputs.ready = readyTimer.hasElapsed(TurretIO.READY_TIME_SEC);

    Logger.processInputs("Turret", inputs);
  }

  public void rotateRight(double rpm) {
    if ((inputs.hardstopDetected && inputs.goingRight)) {
      io.setCurrentAngleDegrees(Constants.turretRightHardstopAngle);
      stop();
      rpm = 0;
    }

    if (inputs.jamDetected
        || Math.abs(inputs.currentAngle)
            >= Math.abs(Constants.turretRightHardstopAngle) - Constants.turretAngleTolerance) {
      rpm = 0;
    }

    setRPM(rpm);
  }

  public void rotateLeft(double rpm) {
    if ((inputs.hardstopDetected && !inputs.goingRight)) {
      io.setCurrentAngleDegrees(Constants.turretLeftHardstopAngle);
      stop();
      rpm = 0;
    }

    if (inputs.jamDetected
        || Math.abs(inputs.currentAngle)
            >= Math.abs(Constants.turretLeftHardstopAngle) - Constants.turretAngleTolerance) {
      rpm = 0;
    }

    setRPM(rpm);
  }

  private void setRPM(double rpm) {
    io.setVelocityRPM(rpm);
  }

  public void stop() {
    io.stop();
  }

  public boolean isReady() {
    return inputs.ready;
  }

  public boolean jamDetected() {
    return inputs.hardstopDetected;
  }

  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }
}
