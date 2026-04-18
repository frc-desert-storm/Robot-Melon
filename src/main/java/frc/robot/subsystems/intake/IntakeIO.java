package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public boolean pivotMotorConnected = false;
    public Angle pivotPosition = Degrees.of(0.0);
    public AngularVelocity pivotVelocity = RPM.of(0.0);
    public Voltage pivotAppliedVolts = Volts.of(0.0);
    public Current pivotCurrentAmps = Amps.of(0.0);
    public Temperature pivotTemp = Celsius.of(0.0);
    public boolean pivotAtGoal = false;

    public boolean pivotLeftMotorConnected = false;
    public Angle pivotLeftPosition = Degrees.of(0.0);
    public AngularVelocity pivotLeftVelocity = RPM.of(0.0);
    public Voltage pivotLeftAppliedVolts = Volts.of(0.0);
    public Current pivotLeftCurrentAmps = Amps.of(0.0);
    public Temperature pivotLeftTemp = Celsius.of(0.0);
    public boolean pivotLeftAtGoal;

    public boolean rollerMotorConnected = false;
    public AngularVelocity rollerVelocity = RPM.of(0.0);
    public Voltage rollerAppliedVolts = Volts.of(0.0);
    public Current rollerCurrentAmps = Amps.of(0.0);
    public Temperature rollerTemp = Celsius.of(0.0);
    public AngularVelocity rollerSetpoint = RPM.of(0.0);
  }

  /** Update logged inputs. Called every robot loop. */
  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPivotAngle(Angle angle) {}

  default void setRollerSpeed(AngularVelocity speed) {}

  default void stopPivot() {}

  default void stopRoller() {}
}
