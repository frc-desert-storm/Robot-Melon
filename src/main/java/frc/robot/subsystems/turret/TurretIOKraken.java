package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class TurretIOKraken implements TurretIO {

  private final TalonFXS motor;

  private final VelocityVoltage velocityReq = new VelocityVoltage(0).withEnableFOC(true);

  private final VoltageOut voltageReq = new VoltageOut(0).withEnableFOC(true);

  private double setpointRPM = 0.0;

  private boolean bGoingRight = false;

  public TurretIOKraken(int id) {

    motor = new TalonFXS(id);

    TalonFXSConfiguration cfg = new TalonFXSConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    cfg.Slot0.kP = 0.07;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12; // replace after SysId
    cfg.Slot0.kS = 0.15;

    cfg.ExternalFeedback.SensorToMechanismRatio = TurretIO.GEAR_RATIO;

    motor.getConfigurator().apply(cfg);

    motor.set(0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    double rpm = (motor.getVelocity().getValueAsDouble() * 60.0);

    inputs.velocityRPM = rpm;
    inputs.velocitySetpointRPM = setpointRPM;

    inputs.currentAngle = getCurrentAngle();

    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.motorTempCelsius = motor.getDeviceTemp().getValueAsDouble();

    inputs.atSetpoint = Math.abs(rpm - setpointRPM) < VELOCITY_TOLERANCE_RPM;

    inputs.hardstopDetected =
        (inputs.torqueCurrentAmps > HARDSTOP_CURRENT_AMPS) && (Math.abs(rpm) < 500);
    inputs.jamDetected = (inputs.torqueCurrentAmps > JAM_CURRENT_AMPS) && (Math.abs(rpm) < 500);

    inputs.ready = false; // computed in subsystem

    inputs.goingRight = bGoingRight;
  }

  // @Override
  // public void rotateRight(double rpm) {
  //   /*setpointRPM = rpm;
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
    motor.setControl(velocityReq.withVelocity(rpm / 60.0));

    bGoingRight = rpm > 0;
  }

  @Override
  public void setCurrentAngleDegrees(double degrees) {
    motor.setPosition(Units.degreesToRotations((degrees)));
  }

  public double getCurrentAngle() {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
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
