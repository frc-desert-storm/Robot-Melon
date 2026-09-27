package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOSim implements IntakeIO {
  private Distance extensionSetpoint = STOW_POSE;
  private AngularVelocity rollerSetpoint = RPM.of(0);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extensionMotorConnected = true;
    inputs.extensionLeftMotorConnected = true;
    inputs.rollerMotorConnected = true;

    inputs.extensionPosition = extensionSetpoint;
    inputs.extensionLeftPosition = extensionSetpoint;
    inputs.extensionVelocity = InchesPerSecond.of(0.0);
    inputs.extensionLeftVelocity = InchesPerSecond.of(0.0);
    inputs.extensionAppliedVolts = Volts.of(0.0);
    inputs.extensionLeftAppliedVolts = Volts.of(0.0);
    inputs.extensionCurrentAmps = Amps.of(0.0);
    inputs.extensionLeftCurrentAmps = Amps.of(0.0);
    inputs.extensionTemp = Celsius.of(0.0);
    inputs.extensionLeftTemp = Celsius.of(0.0);
    inputs.extensionAtGoal = true;
    inputs.extensionLeftAtGoal = true;

    inputs.rollerVelocity = rollerSetpoint;
    inputs.rollerSetpoint = rollerSetpoint;
    inputs.rollerAppliedVolts = Volts.of(0.0);
    inputs.rollerCurrentAmps = Amps.of(0.0);
    inputs.rollerTemp = Celsius.of(0.0);
  }

  @Override
  public void setExtensionDistance(Distance distance) {
    extensionSetpoint = distance;
  }

  @Override
  public void setExtensionVoltage(Voltage voltage) {}

  @Override
  public void zeroExtensionDistance() {
    extensionSetpoint = INTAKING_POSE;
  }

  @Override
  public void setRollerSpeed(AngularVelocity speed) {
    rollerSetpoint = speed;
  }

  @Override
  public void stopExtension() {}

  @Override
  public void stopRoller() {
    rollerSetpoint = RPM.of(0);
  }
}
