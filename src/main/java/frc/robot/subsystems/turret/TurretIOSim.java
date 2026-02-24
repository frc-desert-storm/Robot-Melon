package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class TurretIOSim implements TurretIO {

  private final TalonFX motor; // fake motor, for interface consistency
  private final VelocityVoltage velocityReq = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageReq = new VoltageOut(0).withEnableFOC(true);

  private double simulatedRPM = 0.0;
  private double setpointRPM = 0.0;
  private double appliedVolts = 0.0;
  private double simulatedAngle = 0;

  public TurretIOSim(int id) {
    motor = new TalonFX(id); // still exists for interface, but does nothing
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Simple physics simulation: ramp to setpoint
    double error = setpointRPM - simulatedRPM;
    double ramp = Math.copySign(Math.min(1000.0, Math.abs(error)), error); // max 1000 RPM/sec
    simulatedRPM += ramp * 0.02; // 20ms per update

    inputs.velocityRPM = simulatedRPM;
    // inputs.velocitySetpointRPM = setpointRPM;

    inputs.currentAngle = getCurrentAngle();

    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = 10.0 * Math.abs(appliedVolts / 12.0); // fake current
    inputs.torqueCurrentAmps = 10.0 * Math.abs(appliedVolts / 12.0);
    inputs.motorTempCelsius = 25.0;

    inputs.atSetpoint = Math.abs(simulatedRPM - setpointRPM) < VELOCITY_TOLERANCE_RPM;
    inputs.hardstopDetected = false;
    inputs.ready = false;
  }

  private boolean bGoingRight = false;

  // @Override
  // public void rotateRight(double rpm) {
  //   /*
  //   setpointRPM = rpm;
  //   appliedVolts = Math.min(12.0, rpm / 6000.0 * 12.0); // simple feedforward
  //   motor.setControl(velocityReq.withVelocity(rpm / 60.0));

  //     * spin motor at set rpm
  //     * check if amperage hits critical level
  //     * if it does, stop motor and set encoder accordingly
  //   */
  // }

  // @Override
  // public void rotateLeft(double rpm) {
  //   /*
  //    * spin motor at set rpm
  //    * check if amperage hits critical level
  //    * if it does, stop motor and set encoder accordingly
  //    */
  // }

  // @Override
  // public void rotateToTargetpoint(double degrees) {
  //   /*
  //    * check if encoder value is less than or greater than target
  //    * set motor to spin in the right direction
  //    * stop motor once within tolerable distance of target point
  //    */
  // }

  @Override
  public void setVelocityRPM(double rpm) {
    setpointRPM = rpm;
    appliedVolts = Math.min(12.0, rpm / 6000.0 * 12.0); // simple feedforward
    motor.setControl(velocityReq.withVelocity(rpm / 60.0));
    bGoingRight = rpm > 0;

    if (bGoingRight)
      simulatedAngle +=
          getCurrentAngle() > Constants.turretRightHardstopAngle ? Math.copySign(1, rpm) : 0;
    else
      simulatedAngle +=
          getCurrentAngle() < Constants.turretLeftHardstopAngle ? Math.copySign(1, rpm) : 0;
  }

  public double getCurrentAngle() {
    return simulatedAngle;
  }

  @Override
  public void setVoltage(double volts) {
    setpointRPM = 0.0;
    appliedVolts = volts;
    motor.setControl(voltageReq.withOutput(volts));
  }

  @Override
  public void stop() {
    setpointRPM = 0.0;
    appliedVolts = 0.0;
    motor.stopMotor();
  }

  @Override
  public void setCurrentAngleDegrees(double degrees) {
    // TODO Auto-generated method stub
    simulatedAngle = degrees;
  }
}
