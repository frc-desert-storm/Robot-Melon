package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOKraken implements ShooterIO {

  private final TalonFX motor;

  private final VelocityVoltage velocityReq = new VelocityVoltage(0).withEnableFOC(true);

  private final VoltageOut voltageReq = new VoltageOut(0).withEnableFOC(true);

  private double setpointRPM = 0.0;

  public ShooterIOKraken(int id) {

    motor = new TalonFX(id);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    cfg.Slot0.kP = 0.07;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12; // replace after SysId
    cfg.Slot0.kS = 0.15;

    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    double rpm = motor.getVelocity().getValueAsDouble() * 60.0;

    inputs.velocityRPM = rpm;
    inputs.velocitySetpointRPM = setpointRPM;

    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.motorTempCelsius = motor.getDeviceTemp().getValueAsDouble();

    inputs.atSetpoint = Math.abs(rpm - setpointRPM) < VELOCITY_TOLERANCE_RPM;

    inputs.jamDetected = inputs.torqueCurrentAmps > JAM_CURRENT_AMPS && Math.abs(rpm) < 500;

    inputs.ready = false; // computed in subsystem
  }

  @Override
  public void setVelocityRPM(double rpm) {
    setpointRPM = rpm;
    motor.setControl(velocityReq.withVelocity(rpm / 60.0));
  }

  @Override
  public void setVoltage(double volts) {
    setpointRPM = 0.0;
    motor.setControl(voltageReq.withOutput(volts));
  }

  @Override
  public void stop() {
    setpointRPM = 0.0;
    motor.stopMotor();
  }
}
