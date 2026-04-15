package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public boolean pivotMotorConnected;
    public Angle pivotPosition;
    public AngularVelocity pivotVelocity;
    public Voltage pivotAppliedVolts;
    public Current pivotCurrentAmps;
    public Temperature pivotTemp;
    public boolean pivotAtGoal;

    public boolean pivotLeftMotorConnected;
    public Angle pivotLeftPosition;
    public AngularVelocity pivotLeftVelocity;
    public Voltage pivotLeftAppliedVolts;
    public Current pivotLeftCurrentAmps;
    public Temperature pivotLeftTemp;
    public boolean pivotLeftAtGoal;

    public boolean rollerMotorConnected;
    public AngularVelocity rollerVelocity;
    public Voltage rollerAppliedVolts;
    public Current rollerCurrentAmps;
    public Temperature rollerTemp;
    public AngularVelocity rollerSetpoint;
  }

  /** Update logged inputs. Called every robot loop. */
  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPivotAngle(Angle angle) {}

  default void setRollerSpeed(AngularVelocity speed) {}

  default void stopPivot() {}

  default void stopRoller() {}
}
