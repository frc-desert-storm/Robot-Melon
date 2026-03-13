package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public boolean pivotMotorConnected = false;
    public double pivotPositionRot = 0.0;
    public double pivotVelocityRpm = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
    public double pivotTempCelsius = 0.0;
    public boolean pivotAtGoal = false;

    public boolean rollerMotorConnected = false;
    public double rollerVelocityRpm = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerTempCelsius = 0.0;
    public double rollerRPM = 0.0;
    public double rollerSetpointRPM = 0.0;
  }

  /** Update logged inputs. Called every robot loop. */
  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPivotAngle(Angle angle) {}

  default void setRollerSpeed(AngularVelocity speed) {}

  default void stopPivot() {}

  default void stopRoller() {}
}
