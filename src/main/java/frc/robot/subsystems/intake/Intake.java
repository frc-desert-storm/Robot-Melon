package frc.robot.subsystems.intake;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Angle pivotOffset = Degrees.of(0.0);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command setState(PivotState pivotState, RollerState rollerState) {
    return runOnce(
        () -> {
          this.pivotState = pivotState;
          this.rollerState = rollerState;
          switch (pivotState) {
            case UP -> io.setPivotAngle(Rotations.of(0.0));
            case DOWN -> io.setPivotAngle(Rotations.of(0.360).minus(pivotOffset));
            case IDLE -> io.stopPivot();
          }
          switch (rollerState) {
            case INTAKING -> io.setRollerSpeed(RotationsPerSecond.of(1750.0 / 60));
            case IDLE -> io.stopRoller();
          }
        });
  }

  public void stop() {
    this.rollerState = RollerState.IDLE;
    this.pivotState = PivotState.IDLE;
    io.stopRoller();
    io.stopPivot();
  }

  public PivotState pivotState = PivotState.IDLE;

  public enum PivotState {
    UP,
    DOWN,
    IDLE
  }

  public RollerState rollerState = RollerState.IDLE;

  public enum RollerState {
    INTAKING,
    IDLE
  }
}
