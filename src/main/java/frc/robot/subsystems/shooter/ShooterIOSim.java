package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

  private double measurementStdDevs = 1.0;

  private final FlywheelSim sim;

  private double setpointRPM = 0.0;
  private double appliedVolts = 0.0;

  private final Timer timer = new Timer();
  private double lastTime = 0;

  public ShooterIOSim() {

    var plant =
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1), ShooterIO.gearedInertia, ShooterIO.motorRevolutionsPerRadian);

    sim = new FlywheelSim(plant, DCMotor.getKrakenX60(1), measurementStdDevs);

    timer.start();
    lastTime = timer.get();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    double now = timer.get();
    sim.update(now - lastTime);
    lastTime = now;

    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.velocitySetpointRPM = setpointRPM;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.motorTempCelsius = 25;

    inputs.atSetpoint = Math.abs(inputs.velocityRPM - setpointRPM) < VELOCITY_TOLERANCE_RPM;

    inputs.jamDetected =
        inputs.torqueCurrentAmps > JAM_CURRENT_AMPS && Math.abs(inputs.velocityRPM) < 500;

    inputs.ready = false;
  }

  @Override
  public void setVelocityRPM(double rpm) {
    setpointRPM = rpm;

    double error = rpm - sim.getAngularVelocityRPM();
    appliedVolts = Math.max(-12, Math.min(12, error * 0.002));
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    setpointRPM = 0;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    appliedVolts = 0;
    setpointRPM = 0;
    sim.setInputVoltage(0);
  }
}
