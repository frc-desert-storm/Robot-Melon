package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public boolean extensionMotorConnected = false;
    public Distance extensionPosition = Inches.of(0.0);
    public LinearVelocity extensionVelocity = InchesPerSecond.of(0.0);
    public Voltage extensionAppliedVolts = Volts.of(0.0);
    public Current extensionCurrentAmps = Amps.of(0.0);
    public Temperature extensionTemp = Celsius.of(0.0);
    public boolean extensionAtGoal = false;

    public boolean extensionLeftMotorConnected = false;
    public Distance extensionLeftPosition = Inches.of(0.0);
    public LinearVelocity extensionLeftVelocity = InchesPerSecond.of(0.0);
    public Voltage extensionLeftAppliedVolts = Volts.of(0.0);
    public Current extensionLeftCurrentAmps = Amps.of(0.0);
    public Temperature extensionLeftTemp = Celsius.of(0.0);
    public boolean extensionLeftAtGoal;

    public boolean rollerMotorConnected = false;
    public AngularVelocity rollerVelocity = RPM.of(0.0);
    public Voltage rollerAppliedVolts = Volts.of(0.0);
    public Current rollerCurrentAmps = Amps.of(0.0);
    public Temperature rollerTemp = Celsius.of(0.0);
    public AngularVelocity rollerSetpoint = RPM.of(0.0);
  }

  /** Update logged inputs. Called every robot loop. */
  default void updateInputs(IntakeIOInputs inputs) {}

  default void setExtensionDistance(Distance distance) {}

  default void setExtensionVoltage(Voltage voltage) {}

  default void zeroExtensionDistance() {}

  default void setRollerSpeed(AngularVelocity speed) {}

  default void stopExtension() {}

  default void stopRoller() {}
}
